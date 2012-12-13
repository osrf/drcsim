/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: MultiSenseSL plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b MultiSenseSL plugin broadcasts ROS Image messages
 */

#include "MultiSenseSLPlugin.h"

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
MultiSenseSL::MultiSenseSL()
{
  this->connectionCount = 0;
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1., -1., 10.0, -10.0);
  this->spindleOn = true;
  this->spindleSpeed = 0;
  this->leftCameraFrameRate = 25.0;
  this->rightCameraFrameRate = 25.0;
  this->leftCameraExposureTime = 0.001;
  this->rightCameraExposureTime = 0.001;
  this->leftCameraGain = 1.0;
  this->rightCameraGain = 1.0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
MultiSenseSL::~MultiSenseSL()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

void MultiSenseSL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->drcRobotModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;

  ROS_INFO("Loading MultiSense ROS node.");

  this->lastTime = this->world->GetSimTime();

  this->spindleLink = this->drcRobotModel->GetLink("drc_robot::hokuyo_link");
  if (!this->spindleLink)
    gzerr << "spindle link not found\n";

  this->spindleJoint = this->drcRobotModel->GetJoint("drc_robot::hokuyo_joint");
  if (!this->spindleJoint)
    gzerr << "spindle joint not found\n";

  // sensors::Sensor_V s = sensors::SensorManager::Instance()->GetSensors();
  // for (sensors::Sensor_V::iterator siter = s.begin();
  //                                  siter != s.end(); ++siter)
  //   gzerr << (*siter)->GetName() << "\n";

  this->leftCameraSensor =
    boost::shared_dynamic_cast<sensors::CameraSensor>(
    sensors::SensorManager::Instance()->GetSensor("left_camera_sensor"));
  if (!this->leftCameraSensor)
    gzerr << "left camera sensor not found\n";

  this->rightCameraSensor =
    boost::shared_dynamic_cast<sensors::CameraSensor>(
    sensors::SensorManager::Instance()->GetSensor("right_camera_sensor"));
  if (!this->rightCameraSensor)
    gzerr << "right camera sensor not found\n";

  this->laserSensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("head_hokuyo_sensor"));
  if (!this->laserSensor)
    gzerr << "laser sensor not found\n";

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind( &MultiSenseSL::LoadThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void MultiSenseSL::UpdateStates()
{
  if (this->connectionCount > 0)
  {
    double cur_time = this->world->GetSimTime().Double();

    if (cur_time - this->lastUpdateTime >= 1.0/this->updateRate)
    {
      this->lastUpdateTime = cur_time;
      std_msgs::String msg;
      msg.data = "ok";
      this->pub_status_.publish(msg);
    }
  }

  if (this->spindleOn)
  {
    common::Time curTime = this->world->GetSimTime();
    double dt = (curTime - this->lastTime).Double();
    if (dt > 0)
    {
      // PID control (velocity) spindle
      double spindleError = this->spindleJoint->GetVelocity(0)
                          - this->spindleSpeed;
      double spindleCmd = this->spindlePID.Update(spindleError, dt);
      this->spindleJoint->SetForce(0, spindleCmd);
      this->lastTime = curTime;
    }
  }
  else
  {
    this->spindlePID.Reset();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void MultiSenseSL::LoadThread()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",
      ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

  // ros publication
  ros::AdvertiseOptions pub_status_ao =
    ros::AdvertiseOptions::create<std_msgs::String>(
    "/multisense_sl/status", 10,
    boost::bind(&MultiSenseSL::OnStatusConnect,this),
    boost::bind(&MultiSenseSL::OnStatusDisconnect,this),
    ros::VoidPtr(), &this->queue_);
  this->pub_status_ = this->rosnode_->advertise(pub_status_ao);

  // ros subscription
  ros::SubscribeOptions set_spindle_speed_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/set_spindle_speed", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetSpindleSpeed),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_sub_ =
    this->rosnode_->subscribe(set_spindle_speed_so);

  ros::SubscribeOptions set_spindle_state_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
    "/multisense_sl/set_spindle_state", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Bool::ConstPtr&)>(
        &MultiSenseSL::SetSpindleState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_sub_ =
    this->rosnode_->subscribe(set_spindle_state_so);

  ros::SubscribeOptions set_left_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/left/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetLeftCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_left_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_left_camera_frame_rate_so);

  ros::SubscribeOptions set_right_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/right/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetRightCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_right_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_right_camera_frame_rate_so);

  ros::SubscribeOptions set_left_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/left/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetLeftCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_left_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_left_camera_exposure_time_so);

  ros::SubscribeOptions set_right_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/right/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetRightCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_right_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_right_camera_exposure_time_so);

  ros::SubscribeOptions set_left_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/left/set_camera_gain", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetLeftCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_left_camera_gain_sub_ =
    this->rosnode_->subscribe(set_left_camera_gain_so);

  ros::SubscribeOptions set_right_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/multisense_sl/right/set_camera_gain", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetRightCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_right_camera_gain_sub_ =
    this->rosnode_->subscribe(set_right_camera_gain_so);

  // Advertise services on the custom queue
  std::string set_spindle_speed_service_name(
    "/multisense_sl/set_spindle_speed");
  ros::AdvertiseServiceOptions set_spindle_speed_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_speed_service_name,
      boost::bind(&MultiSenseSL::SetSpindleSpeed,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_service_ =
    this->rosnode_->advertiseService(set_spindle_speed_aso);

  std::string set_spindle_state_service_name(
    "/multisense_sl/set_spindle_state");
  ros::AdvertiseServiceOptions set_spindle_state_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_state_service_name,
      boost::bind(&MultiSenseSL::SetSpindleState,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_service_ =
    this->rosnode_->advertiseService(set_spindle_state_aso);


  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &MultiSenseSL::QueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&MultiSenseSL::UpdateStates, this));
}

void MultiSenseSL::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void MultiSenseSL::OnStatusConnect()
{
  this->connectionCount++;
}

void MultiSenseSL::OnStatusDisconnect()
{
  this->connectionCount--;
}

bool MultiSenseSL::SetSpindleSpeed(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

bool MultiSenseSL::SetSpindleState(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

void MultiSenseSL::SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  this->spindleSpeed = (double)_msg->data;
}

void MultiSenseSL::SetSpindleState(const std_msgs::Bool::ConstPtr &_msg)
{
  this->spindleOn = (double)_msg->data;
}

void MultiSenseSL::SetLeftCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->leftCameraFrameRate = (double)_msg->data;
  this->leftCameraSensor->SetUpdateRate(this->leftCameraFrameRate);
}

void MultiSenseSL::SetRightCameraFrameRate(const std_msgs::Float64::ConstPtr
                                           &_msg)
{
  this->rightCameraFrameRate = (double)_msg->data;
  this->rightCameraSensor->SetUpdateRate(this->rightCameraFrameRate);
}

void MultiSenseSL::SetLeftCameraExposureTime(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->leftCameraExposureTime = (double)_msg->data;
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

void MultiSenseSL::SetRightCameraExposureTime(const std_msgs::Float64::ConstPtr
                                           &_msg)
{
  this->rightCameraExposureTime = (double)_msg->data;
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

void MultiSenseSL::SetLeftCameraGain(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->leftCameraGain = (double)_msg->data;
  gzwarn << "setting camera gain in sim not implemented\n";
}

void MultiSenseSL::SetRightCameraGain(const std_msgs::Float64::ConstPtr
                                           &_msg)
{
  this->rightCameraGain = (double)_msg->data;
  gzwarn << "setting camera gain in sim not implemented\n";
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MultiSenseSL)

}
