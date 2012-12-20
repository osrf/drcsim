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
#ifndef GAZEBO_DRC_ROBOT_PLUGIN_HH
#define GAZEBO_DRC_ROBOT_PLUGIN_HH

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
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "gazebo/sensors/Sensor.hh"

#include "boost/thread/mutex.hpp"

namespace gazebo
{
  class DRCRobotPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DRCRobotPlugin();

    /// \brief Destructor
    public: virtual ~DRCRobotPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: event::ConnectionPtr rContactUpdateConnection;
    private: event::ConnectionPtr lContactUpdateConnection;
    void OnLContactUpdate();
    void OnRContactUpdate();

    /// Throttle update rate
    private: double lastStatusTime;
    private: double updateRate;

    // Contact sensors
    private: sensors::ContactSensorPtr lFootContactSensor;
    private: sensors::ContactSensorPtr rFootContactSensor;
    private: ros::Publisher pubLFootContact;
    private: ros::Publisher pubRFootContact;

    // Force torque sensors at ankles
    private: physics::JointPtr rAnkleJoint;
    private: physics::JointPtr lAnkleJoint;
    private: ros::Publisher pubLAnkleFT;
    private: ros::Publisher pubRAnkleFT;

    // Force torque sensors at the wrists
    private: physics::JointPtr rWristJoint;
    private: physics::JointPtr lWristJoint;
    private: ros::Publisher pubLWristFT;
    private: ros::Publisher pubRWristFT;

    // IMU sensor
    private: std::string imuLinkName;
    private: physics::LinkPtr imuLink;
    private: common::Time lastImuTime;
    private: math::Pose imuReferencePose;
    private: math::Vector3 imuLastLinearVel;
    private: ros::Publisher pubImu;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferredLoadThread;

    // reset of ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue queue;
    private: void QueueThread();
    private: boost::thread callbackQueeuThread;
    private: ros::Publisher pubStatus;
    private: math::Vector3 lFootForce;
    private: math::Vector3 lFootTorque;
    private: math::Vector3 rFootForce;
    private: math::Vector3 rFootTorque;

    private: physics::Joint_V joints;

    // Controls stuff
    private: common::Time lastControllerUpdateTime;
  };
/** \} */
/// @}
}
#endif
