/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
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
 * Desc: Plugin for controlling MultiSense SL model in gazebo
 * Author: John Hsu
 */
#ifndef __MULTISENSE_SL_PLUGIN_HH_
#define __MULTISENSE_SL_PLUGIN_HH_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include "std_srvs/Empty.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "transport/TransportTypes.hh"
#include "gazebo/physics/physics.hh"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

  class MultiSenseSL : public ModelPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model
    public: MultiSenseSL();

    /// \brief Destructor
    public: ~MultiSenseSL();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateStates();

    /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// ROS connection count
    private: int connectionCount;
    private: void OnStatusConnect();
    private: void OnStatusDisconnect();

    // deferred ros loading
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // reset of ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // ros topics
    private: ros::Publisher pub_status_;

    // ros services
    private: ros::ServiceServer set_spindle_state_service_;
    private: bool SetSpindleState(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    private: ros::ServiceServer set_spindle_speed_service_;
    private: bool SetSpindleSpeed(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    // gazebo variables
    private: physics::WorldPtr world;
    private: physics::ModelPtr drcRobotModel;
    private: physics::LinkPtr spindleLink;
    private: physics::JointPtr spindleJoint;
    private: sdf::ElementPtr sdf;

    /// Throttle update rate
    private: double lastUpdateTime;
    private: double updateRate;

  };

}
#endif

