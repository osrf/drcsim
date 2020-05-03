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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <math.h>
#include "DRCVehicleROSPlugin.h"
#include "gazebo/common/common.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/SphereShape.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehicleROSPlugin::DRCVehicleROSPlugin()
{
  this->rosPublishPeriod = common::Time(1.0);
  this->lastRosPublishTime = common::Time(0.0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehicleROSPlugin::~DRCVehicleROSPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->ros_publish_connection_);
  this->rosNode->shutdown();
  this->queue.clear();
  this->queue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize
void DRCVehicleROSPlugin::Init()
{
}


////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetKeyState(const std_msgs::Int8::ConstPtr &_msg)
{
  if (_msg->data == 0)
    this->SetKeyOff();
  else if (_msg->data == 1)
    this->SetKeyOn();
  else
    ROS_ERROR("Invalid Key State: %d, expected 0 or 1\n",
      static_cast<int16_t>(_msg->data));
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetDirectionState(const std_msgs::Int8::ConstPtr &_msg)
{
  if (_msg->data == 0)
    this->DRCVehiclePlugin::SetDirectionState(NEUTRAL);
  else if (_msg->data == 1)
    this->DRCVehiclePlugin::SetDirectionState(FORWARD);
  else if (_msg->data == -1)
    this->DRCVehiclePlugin::SetDirectionState(REVERSE);
  else
    ROS_ERROR("Invalid Direction State: %d, expected -1, 0, or 1\n",
      static_cast<int16_t>(_msg->data));
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandBrakeState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  DRCVehiclePlugin::SetHandBrakeState( static_cast<double>(_msg->data) );
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandWheelState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  DRCVehiclePlugin::SetHandWheelState( static_cast<double>(_msg->data) );
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetGasPedalState(const std_msgs::Float64::ConstPtr
                                                &_msg)
{
  DRCVehiclePlugin::SetGasPedalState( static_cast<double>(_msg->data) );
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetBrakePedalState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  DRCVehiclePlugin::SetBrakePedalState( static_cast<double>(_msg->data) );
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCVehicleROSPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  DRCVehiclePlugin::Load(_parent, _sdf);

  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  ros::SubscribeOptions hand_wheel_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->model->GetName() + "/hand_wheel/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetHandWheelState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subHandWheelCmd = this->rosNode->subscribe(hand_wheel_cmd_so);

  ros::SubscribeOptions hand_brake_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->model->GetName() + "/hand_brake/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetHandBrakeState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subHandBrakeCmd = this->rosNode->subscribe(hand_brake_cmd_so);

  ros::SubscribeOptions gas_pedal_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->model->GetName() + "/gas_pedal/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetGasPedalState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subGasPedalCmd = this->rosNode->subscribe(gas_pedal_cmd_so);

  ros::SubscribeOptions brake_pedal_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->model->GetName() + "/brake_pedal/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetBrakePedalState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subBrakePedalCmd = this->rosNode->subscribe(brake_pedal_cmd_so);

  ros::SubscribeOptions key_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Int8>(
    this->model->GetName() + "/key/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Int8::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetKeyState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subKeyCmd = this->rosNode->subscribe(key_cmd_so);

  ros::SubscribeOptions direction_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Int8>(
    this->model->GetName() + "/direction/cmd", 100,
    boost::bind( static_cast<void (DRCVehicleROSPlugin::*)
      (const std_msgs::Int8::ConstPtr&)>(
        &DRCVehicleROSPlugin::SetDirectionState),this,_1),
    ros::VoidPtr(), &this->queue);
  this->subDirectionCmd = this->rosNode->subscribe(direction_cmd_so);

  this->pubHandWheelState = this->rosNode->advertise<std_msgs::Float64>(
    this->model->GetName() + "/hand_wheel/state",10);
  this->pubHandBrakeState = this->rosNode->advertise<std_msgs::Float64>(
    this->model->GetName() + "/hand_brake/state",10);
  this->pubGasPedalState = this->rosNode->advertise<std_msgs::Float64>(
    this->model->GetName() + "/gas_pedal/state",10);
  this->pubBrakePedalState = this->rosNode->advertise<std_msgs::Float64>(
    this->model->GetName() + "/brake_pedal/state",10);
  this->pubKeyState = this->rosNode->advertise<std_msgs::Int8>(
    this->model->GetName() + "/key/state",10);
  this->pubDirectionState = this->rosNode->advertise<std_msgs::Int8>(
    this->model->GetName() + "/direction/state",10);

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind( &DRCVehicleROSPlugin::QueueThread,this ) );

  this->ros_publish_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehicleROSPlugin::RosPublishStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// Returns the ROS publish period (seconds).
common::Time DRCVehicleROSPlugin::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}

////////////////////////////////////////////////////////////////////////////////
// Set the ROS publish frequency (Hz).
void DRCVehicleROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    this->rosPublishPeriod = 1.0/_hz;
  else
    this->rosPublishPeriod = 0.0;  
}

////////////////////////////////////////////////////////////////////////////////
// Publish hand wheel, gas pedal, and brake pedal on ROS
void DRCVehicleROSPlugin::RosPublishStates()
{
  if (this->world->GetSimTime() - this->lastRosPublishTime >=
      this->rosPublishPeriod)
  {
    // Update time
    this->lastRosPublishTime = this->world->GetSimTime();
    // Publish Float64 messages
    std_msgs::Float64 msg_steer, msg_brake, msg_gas, msg_hand_brake;
    msg_steer.data = GetHandWheelState();
    this->pubHandWheelState.publish(msg_steer);
    msg_brake.data = GetBrakePedalState();
    this->pubBrakePedalState.publish(msg_brake);
    msg_gas.data = GetGasPedalState();
    this->pubGasPedalState.publish(msg_gas);
    msg_hand_brake.data = GetHandBrakeState();
    this->pubHandBrakeState.publish(msg_hand_brake);
    // Publish Int8
    std_msgs::Int8 msg_key, msg_direction;
    msg_key.data = static_cast<int8_t>(GetKeyState());
    this->pubKeyState.publish(msg_key);
    msg_direction.data = static_cast<int8_t>(GetDirectionState());
    this->pubDirectionState.publish(msg_direction);
  }
}

void DRCVehicleROSPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}


GZ_REGISTER_MODEL_PLUGIN(DRCVehicleROSPlugin)
}

