/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2012 Open Source Robotics Foundation
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
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MultiSenseSL)

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::MultiSenseSL()
{
  /// \todo: hardcoded for now, make them into plugin parameters
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1., -1., 10.0, -10.0);
  this->spindleOn = true;
  this->spindleSpeed = 0;
  this->spindleMaxRPM = 50.0;
  this->spindleMinRPM = 0;
  this->multiCameraFrameRate = 25.0;
  this->multiCameraExposureTime = 0.001;
  this->multiCameraGain = 1.0;
}

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::~MultiSenseSL()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->atlasModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;

  ROS_INFO("Loading MultiSense ROS node.");

  this->lastTime = this->world->GetSimTime();

  this->spindleLink = this->atlasModel->GetLink("atlas::hokuyo_link");
  if (!this->spindleLink)
  {
    gzerr << "spindle link not found, plugin will stop loading\n";
    return;
  }

  this->spindleJoint = this->atlasModel->GetJoint("atlas::hokuyo_joint");
  if (!this->spindleJoint)
  {
    gzerr << "spindle joint not found, plugin will stop loading\n";
    return;
  }

  // sensors::Sensor_V s = sensors::SensorManager::Instance()->GetSensors();
  // for (sensors::Sensor_V::iterator siter = s.begin();
  //                                  siter != s.end(); ++siter)
  //   gzerr << (*siter)->GetName() << "\n";

  this->multiCameraSensor =
    boost::shared_dynamic_cast<sensors::MultiCameraSensor>(
    sensors::SensorManager::Instance()->GetSensor("stereo_camera"));
  if (!this->multiCameraSensor)
    gzerr << "multicamera sensor not found\n";

  this->laserSensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("head_hokuyo_sensor"));
  if (!this->laserSensor)
    gzerr << "laser sensor not found\n";

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }
  
  this->deferred_load_thread_ = boost::thread(
    boost::bind( &MultiSenseSL::LoadThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::LoadThread()
{
  // create ros node
  this->rosnode_ = new ros::NodeHandle("");

  // ros subscription
  ros::SubscribeOptions set_spindle_speed_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_spindle_speed", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetSpindleSpeed),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_sub_ =
    this->rosnode_->subscribe(set_spindle_speed_so);

  /* not implemented, not supported
  ros::SubscribeOptions set_spindle_state_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
    "multisense_sl/set_spindle_state", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Bool::ConstPtr&)>(
        &MultiSenseSL::SetSpindleState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_sub_ =
    this->rosnode_->subscribe(set_spindle_state_so);

  ros::SubscribeOptions set_multi_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so);

  ros::SubscribeOptions set_multi_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_multi_camera_exposure_time_so);

  ros::SubscribeOptions set_multi_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_gain", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_gain_sub_ =
    this->rosnode_->subscribe(set_multi_camera_gain_so);
  */

  /// \todo: waiting for gen_srv to be implemented (issue #37)
  /* Advertise services on the custom queue
  std::string set_spindle_speed_service_name(
    "multisense_sl/set_spindle_speed");
  ros::AdvertiseServiceOptions set_spindle_speed_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_speed_service_name,
      boost::bind(&MultiSenseSL::SetSpindleSpeed,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_service_ =
    this->rosnode_->advertiseService(set_spindle_speed_aso);

  std::string set_spindle_state_service_name(
    "multisense_sl/set_spindle_state");
  ros::AdvertiseServiceOptions set_spindle_state_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_state_service_name,
      boost::bind(&MultiSenseSL::SetSpindleState,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_service_ =
    this->rosnode_->advertiseService(set_spindle_state_aso);
  */

  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &MultiSenseSL::QueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&MultiSenseSL::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::UpdateStates()
{
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
void MultiSenseSL::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleSpeed(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleState(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  this->spindleSpeed = (double)_msg->data;
  if (this->spindleSpeed > this->spindleMaxRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMaxRPM * 2.0*M_PI / 60.0;
  else if (this->spindleSpeed < this->spindleMinRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMinRPM * 2.0*M_PI / 60.0;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleState(const std_msgs::Bool::ConstPtr &_msg)
{
  this->spindleOn = (double)_msg->data;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraFrameRate = (double)_msg->data;
  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraExposureTime(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraExposureTime = (double)_msg->data;
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraGain = (double)_msg->data;
  gzwarn << "setting camera gain in sim not implemented\n";
}
}
