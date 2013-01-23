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
 * Desc: Plugin for the SandiaHand
 * Author: John Hsu
 * Date: December 2012
 */
#ifndef GAZEBO_SANDIA_HAND_PLUGIN_HH
#define GAZEBO_SANDIA_HAND_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>

#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "gazebo/sensors/Sensor.hh"

#include "boost/thread/mutex.hpp"

#include "osrf_msgs/JointCommands.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{
  class SandiaHandPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SandiaHandPlugin();

    /// \brief Destructor
    public: virtual ~SandiaHandPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();
    private: void CopyVectorIfValid(const std::vector<double> &from,
                                    std::vector<double> &to,
                                    const unsigned joint_offset);

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: double lastStatusTime;
    private: double updateRate;

    // IMU sensor
    private: common::Time lastImuTime;
    private: std::string leftImuLinkName;
    private: physics::LinkPtr leftImuLink;
    private: math::Pose leftImuReferencePose;
    private: math::Vector3 leftImuLastLinearVel;
    private: ros::Publisher pubLeftImu;
    private: std::string rightImuLinkName;
    private: physics::LinkPtr rightImuLink;
    private: math::Pose rightImuReferencePose;
    private: math::Vector3 rightImuLastLinearVel;
    private: ros::Publisher pubRightImu;

    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;
    private: ros::Publisher pubLeftJointStates;
    private: ros::Publisher pubRightJointStates;

    private: ros::Subscriber subJointCommands[2];
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg,
      const unsigned jointOffset); // to handle left/right hands

    private: physics::Joint_V joints;
    private: class ErrorTerms
      {
        double q_p;
        double d_q_p_dt;
        double q_i;
        double qd_p;
        friend class SandiaHandPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState leftJointStates;
    private: sensor_msgs::JointState rightJointStates;

    // Controls stuff
    private: common::Time lastControllerUpdateTime;
  };
/** \} */
/// @}
}
#endif
