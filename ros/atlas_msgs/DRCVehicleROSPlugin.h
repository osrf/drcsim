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

#ifndef GAZEBO_DRC_VEHICLE_ROS_PLUGIN_HH
#define GAZEBO_DRC_VEHICLE_ROS_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include "DRCVehiclePlugin.hh"

namespace gazebo
{
  class DRCVehicleROSPlugin: public DRCVehiclePlugin
  {
    /// \brief Constructor.
    public: DRCVehicleROSPlugin();

    /// \brief Destructor.
    public: virtual ~DRCVehicleROSPlugin();

    /// \brief Load the controller.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish the steering and pedal states on ROS topics.
    private: void RosPublishStates();

    /// \brief Sets the state of the key switch.
    /// \param[in] _msg Desired key state as Int8 message.
    ///            Use 0 for OFF, 1 for ON.
    public: void SetKeyState(const std_msgs::Int8::ConstPtr &_msg);

    /// \brief Sets the state of the direction switch.
    /// \param[in] _msg Desired direction state as Int8 message.
    ///            Use -1 for REVERSE, 0 for NEUTRAL, 1 for FORWARD.
    public: void SetDirectionState(const std_msgs::Int8::ConstPtr &_msg);

    /// \brief Set the steering wheel angle; this will also update the front
    ///        wheel steering angle.
    /// \param[in] _msg ROS std_msgs::Float64 message.
    public: void SetHandWheelState(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Set the steering wheel angle; this will also update the front
    ///        wheel steering angle.
    /// \param[in] _msg ROS std_msgs::Float64 message.
    public: void SetHandBrakeState(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Specify gas pedal position in meters.
    /// \param[in] _msg ROS std_msgs::Float64 message.
    public: void SetGasPedalState(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Specify brake pedal position in meters.
    /// \param[in] _msg ROS std_msgs::Float64 message.
    public: void SetBrakePedalState(const std_msgs::Float64::ConstPtr &_msg);

    /// Returns the ROS publish period (seconds).
    public: common::Time GetRosPublishPeriod();

    /// Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// Default plugin init call.
    public: void Init();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the publish event connection.
    private: event::ConnectionPtr ros_publish_connection_;

    // ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue queue;
    private: void QueueThread();
    private: boost::thread callbackQueueThread;
    private: ros::Publisher pubBrakePedalState;
    private: ros::Publisher pubGasPedalState;
    private: ros::Publisher pubHandWheelState;
    private: ros::Publisher pubHandBrakeState;
    private: ros::Publisher pubKeyState;
    private: ros::Publisher pubDirectionState;
    private: ros::Subscriber subBrakePedalCmd;
    private: ros::Subscriber subGasPedalCmd;
    private: ros::Subscriber subHandWheelCmd;
    private: ros::Subscriber subHandBrakeCmd;
    private: ros::Subscriber subKeyCmd;
    private: ros::Subscriber subDirectionCmd;
    private: common::Time rosPublishPeriod;
    private: common::Time lastRosPublishTime;
  };
/** \} */
/// @}
}
#endif
