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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_DRC_VEHICLE_ROS_PLUGIN_HH
#define GAZEBO_DRC_VEHICLE_ROS_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include "plugins/DRCVehiclePlugin.hh"

namespace gazebo
{
  class DRCVehiclePlugin;
  
  class DRCVehicleROSPlugin: public ModelPlugin
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
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;
    private: ros::Publisher brake_pedal_state_pub_;
    private: ros::Publisher gas_pedal_state_pub_;
    private: ros::Publisher hand_wheel_state_pub_;
    private: ros::Publisher hand_brake_state_pub_;
    private: ros::Publisher key_state_pub_;
    private: ros::Publisher direction_state_pub_;
    private: ros::Subscriber brake_pedal_cmd_sub_;
    private: ros::Subscriber gas_pedal_cmd_sub_;
    private: ros::Subscriber hand_wheel_cmd_sub_;
    private: ros::Subscriber hand_brake_cmd_sub_;
    private: ros::Subscriber key_cmd_sub_;
    private: ros::Subscriber direction_cmd_sub_;
    private: common::Time rosPublishPeriod;
    private: common::Time lastRosPublishTime;
  };
/** \} */
/// @}
}
#endif
