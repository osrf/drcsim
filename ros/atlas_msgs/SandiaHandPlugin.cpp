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
   Desc: Sandia Hand plugin for interfacing with Sandia Hand gazebo simulation
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b Sandia Hand plugin ROS API on wiki
 */

#include "SandiaHandPlugin.h"

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SandiaHandPlugin)

////////////////////////////////////////////////////////////////////////////////
SandiaHandPlugin::SandiaHandPlugin()
{
  /// \todo: hardcoded for now, make them into plugin parameters
  this->leftCameraFrameRate = 25.0;
  this->rightCameraFrameRate = 25.0;
}

////////////////////////////////////////////////////////////////////////////////
SandiaHandPlugin::~SandiaHandPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->atlasModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;

  ROS_INFO("Loading Sandia Hand Plugin ROS node.");

  this->lastTime = this->world->GetSimTime();

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

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }
  
  this->deferred_load_thread_ = boost::thread(
    boost::bind( &SandiaHandPlugin::LoadThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::LoadThread()
{
  // create ros node
  this->rosnode_ = new ros::NodeHandle("");

  // ros publication
  this->pub_status_ = this->rosnode_->advertise<std_msgs::String>(
    "sandia_hand/status", 10);

  // ros subscription
  ros::SubscribeOptions set_left_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "sandia_hand/left/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (SandiaHandPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SandiaHandPlugin::SetLeftCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_left_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_left_camera_frame_rate_so);

  ros::SubscribeOptions set_right_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "sandia_hand/right/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (SandiaHandPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SandiaHandPlugin::SetRightCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_right_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_right_camera_frame_rate_so);

  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &SandiaHandPlugin::QueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&SandiaHandPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::UpdateStates()
{
  if (this->pub_status_.getNumSubscribers() > 0)
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
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::SetLeftCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->leftCameraFrameRate = (double)_msg->data;
  this->leftCameraSensor->SetUpdateRate(this->leftCameraFrameRate);
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::SetRightCameraFrameRate(const std_msgs::Float64::ConstPtr
                                           &_msg)
{
  this->rightCameraFrameRate = (double)_msg->data;
  this->rightCameraSensor->SetUpdateRate(this->rightCameraFrameRate);
}
}
