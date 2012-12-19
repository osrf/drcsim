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
    private: double lastUpdateTime;
    private: double updateRate;

    // Contact sensors
    private: sensors::ContactSensorPtr lFootContactSensor;
    private: sensors::ContactSensorPtr rFootContactSensor;
    private: physics::JointPtr rFootJoint;
    private: physics::JointPtr lFootJoint;
    private: ros::Publisher pub_l_foot_ft_;
    private: ros::Publisher pub_r_foot_ft_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // reset of ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;
    private: ros::Publisher pub_status_;
    private: ros::Publisher pub_l_foot_contact_;
    private: ros::Publisher pub_r_foot_contact_;
    private: math::Vector3 lFootForce;
    private: math::Vector3 lFootTorque;
    private: math::Vector3 rFootForce;
    private: math::Vector3 rFootTorque;
  };
/** \} */
/// @}
}
#endif
