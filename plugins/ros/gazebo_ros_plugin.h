#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

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

/* Desc: External interfaces for Gazebo
 * Author: John Hsu adapted original gazebo main.cc by Nate Koenig
 * Date: 25 Apr 2010
 * SVN: $Id: main.cc 8598 2010-03-22 21:59:24Z hsujohnhsu $
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <tinyxml.h>

#include "Server.hh"
#include "physics/Physics.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/Entity.hh"
#include "physics/Collision.hh"
#include "physics/Inertial.hh"
#include "physics/Base.hh"
#include "physics/Link.hh"
#include "physics/Model.hh"
#include "physics/Joint.hh"
#include "common/CommonTypes.hh"
#include "common/Exception.hh"
#include "common/SystemPaths.hh"
#include "common/Plugin.hh"
#include "transport/Node.hh"
//#include "msgs/MessageTypes.hh" // implicitly included from CommonTypes.hh

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>

// Services
#include "std_srvs/Empty.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// For model pose transform to set custom joint angles
#include <ros/ros.h>
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include <boost/shared_ptr.hpp>

#include <boost/algorithm/string.hpp>

namespace gazebo
{

class GazeboRosApiPlugin : public SystemPlugin
{
  public:
    GazeboRosApiPlugin();
    ~GazeboRosApiPlugin();

    void Load(int argc, char** argv);

    /// \brief ros queue thread for this node
    void gazeboQueueThread();

    /// \brief advertise services
    void AdvertiseServices();

    void onLinkStatesConnect();
    void onModelStatesConnect();
    void onLinkStatesDisconnect();
    void onModelStatesDisconnect();


    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    void spin();

  private:
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue gazebo_queue_;
    boost::thread* gazebo_callback_queue_thread_;

    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr time_update_event_;
    gazebo::event::ConnectionPtr load_gazebo_ros_api_plugin_event_;

    ros::ServiceServer reset_simulation_service_;
    ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;

    // for internal gazebo xml use
    std::string xmlPrefix_;
    std::string xmlSuffix_;

    boost::thread* ros_spin_thread_;


    ros::Publisher     pub_clock_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private: boost::mutex lock_;

    /// \brief utilites for checking incoming string URDF/XML/Param
    bool IsURDF(std::string model_xml);
    bool IsGazeboModelXML(std::string model_xml);
    bool IsSDF(std::string model_xml);
    void LoadGazeboRosApiPlugin(std::string _worldName);
    bool world_created_;

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void wrenchBodySchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void forceJointSchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void publishSimTime();

    std::string robot_namespace_;

};
}
#endif
