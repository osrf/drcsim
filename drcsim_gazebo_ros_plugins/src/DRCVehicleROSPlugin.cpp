/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <math.h>
#include <stdlib.h>
#include "drcsim_gazebo_ros_plugins/DRCVehicleROSPlugin.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/Base.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehicleROSPlugin::DRCVehicleROSPlugin()
{
  this->rosPublishPeriod = common::Time(0.05);
  this->lastRosPublishTime = common::Time(0.0);
  this->rosNode = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehicleROSPlugin::~DRCVehicleROSPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->ros_publish_connection_);
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
  DRCVehiclePlugin::Init();
}

////////////////////////////////////////////////////////////////////////////////
// Reset
void DRCVehicleROSPlugin::Reset()
{
  this->lastRosPublishTime.Set(0, 0);
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
void DRCVehicleROSPlugin::SetDirectionState(
  const std_msgs::Int8::ConstPtr &_msg)
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

  this->UpdateFNRSwitchTime();
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandBrakePercent(const std_msgs::Float64::ConstPtr
    &_msg)
{
  double min, max, percent, cmd;
  percent = math::clamp(static_cast<double>(_msg->data), 0.0, 1.0);
  DRCVehiclePlugin::GetHandBrakeLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetHandBrakeState(cmd);
  this->UpdateHandBrakeTime();
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandWheelState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  DRCVehiclePlugin::SetHandWheelState(static_cast<double>(_msg->data));
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetGasPedalPercent(const std_msgs::Float64::ConstPtr
                                                &_msg)
{
  double min, max, percent, cmd;
  percent = math::clamp(static_cast<double>(_msg->data), 0.0, 1.0);
  DRCVehiclePlugin::GetGasPedalLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetGasPedalState(cmd);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetBrakePedalPercent(const std_msgs::Float64::ConstPtr
    &_msg)
{
  double min, max, percent, cmd;
  percent = math::clamp(static_cast<double>(_msg->data), 0.0, 1.0);
  DRCVehiclePlugin::GetBrakePedalLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetBrakePedalState(cmd);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCVehicleROSPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // By default, cheats are off.  Allow override via environment variable.
  char* cheatsEnabledString = getenv("VRC_CHEATS_ENABLED");
  if (cheatsEnabledString && (std::string(cheatsEnabledString) == "1"))
    this->cheatsEnabled = true;
  else
    this->cheatsEnabled = false;

  try
  {
  DRCVehiclePlugin::Load(_parent, _sdf);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your vehicle model is correct and up-to-date."
          << '\n';
    return;
  }

  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  if (this->cheatsEnabled)
  {
    ros::SubscribeOptions hand_wheel_cmd_so =
      ros::SubscribeOptions::create<std_msgs::Float64>(
      this->model->GetName() + "/hand_wheel/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetHandWheelState), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subHandWheelCmd = this->rosNode->subscribe(hand_wheel_cmd_so);
  
    ros::SubscribeOptions hand_brake_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      this->model->GetName() + "/hand_brake/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetHandBrakePercent), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subHandBrakeCmd = this->rosNode->subscribe(hand_brake_cmd_so);
  
    ros::SubscribeOptions gas_pedal_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      this->model->GetName() + "/gas_pedal/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetGasPedalPercent), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subGasPedalCmd = this->rosNode->subscribe(gas_pedal_cmd_so);
  
    ros::SubscribeOptions brake_pedal_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      this->model->GetName() + "/brake_pedal/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetBrakePedalPercent), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subBrakePedalCmd = this->rosNode->subscribe(brake_pedal_cmd_so);
  
    ros::SubscribeOptions key_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Int8 >(
      this->model->GetName() + "/key/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Int8::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetKeyState), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subKeyCmd = this->rosNode->subscribe(key_cmd_so);
  
    ros::SubscribeOptions direction_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Int8 >(
      this->model->GetName() + "/direction/cmd", 100,
      boost::bind(static_cast< void (DRCVehicleROSPlugin::*)
        (const std_msgs::Int8::ConstPtr&) >(
          &DRCVehicleROSPlugin::SetDirectionState), this, _1),
      ros::VoidPtr(), &this->queue);
    this->subDirectionCmd = this->rosNode->subscribe(direction_cmd_so);

    this->pubHandWheelState = this->rosNode->advertise<std_msgs::Float64>(
      this->model->GetName() + "/hand_wheel/state", 10);
    this->pubHandBrakeState = this->rosNode->advertise<std_msgs::Float64>(
      this->model->GetName() + "/hand_brake/state", 10);
    this->pubGasPedalState = this->rosNode->advertise<std_msgs::Float64>(
      this->model->GetName() + "/gas_pedal/state", 10);
    this->pubBrakePedalState = this->rosNode->advertise<std_msgs::Float64>(
      this->model->GetName() + "/brake_pedal/state", 10);
    this->pubKeyState = this->rosNode->advertise<std_msgs::Int8>(
      this->model->GetName() + "/key/state", 10);
    this->pubDirectionState = this->rosNode->advertise<std_msgs::Int8>(
      this->model->GetName() + "/direction/state", 10);

    // ros callback queue for processing subscription
    this->callbackQueueThread = boost::thread(
      boost::bind(&DRCVehicleROSPlugin::QueueThread, this));

    this->ros_publish_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DRCVehicleROSPlugin::RosPublishStates, this));
  }
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
    msg_brake.data = GetBrakePedalPercent();
    this->pubBrakePedalState.publish(msg_brake);
    msg_gas.data = GetGasPedalPercent();
    this->pubGasPedalState.publish(msg_gas);
    msg_hand_brake.data = GetHandBrakePercent();
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

