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
 * Desc: Bumper Controller
 * Author: Nate Koenig mod by John Hsu
 * Date: 24 Sept 2008
 */
#ifndef GAZEBO_ROS_BUMPER_HH
#define GAZEBO_ROS_BUMPER_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sys/time.h>

#include "sdf/interface/Param.hh"
#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "sensors/SensorTypes.hh"
#include "sensors/ContactSensor.hh"
#include "plugins/ContactPlugin.hh"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

namespace gazebo
{
  /// \brief A Bumper controller
  class GazeboRosBumper : public ContactPlugin
  {
    /// Constructor
    public: GazeboRosBumper();
  
    /// Destructor
    public: ~GazeboRosBumper();
  
    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  
    /// Update the controller
    protected: virtual void UpdateChild();
  

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher contact_pub_;

    /// \brief set topic name of broadcast
    private: std::string bumper_topic_name_;

    private: std::string frame_name_;

    /// \brief broadcast some string for now.
    private: gazebo_msgs::ContactsState contact_state_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue contact_queue_;
    private: void ContactQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
  };

}

#endif

