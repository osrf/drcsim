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

#include "common/Events.hh"
#include "gazebo_ros_api_plugin.h"

namespace gazebo
{

    GazeboRosApiPlugin::GazeboRosApiPlugin()
    {
      this->robot_namespace_.clear();
      this->world_created_ = false;
    }

    GazeboRosApiPlugin::~GazeboRosApiPlugin()
    {
      // shutdown ros
      this->rosnode_->shutdown();
      delete this->rosnode_;

      // shutdown ros queue
      this->gazebo_callback_queue_thread_->join();
      delete this->gazebo_callback_queue_thread_;

      this->ros_spin_thread_->join();
      delete this->ros_spin_thread_;
    }

    void GazeboRosApiPlugin::Load(int argc, char** argv)
    {
      // setup ros related
      if (!ros::isInitialized())
        ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
      else
        ROS_ERROR("Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

      this->rosnode_ = new ros::NodeHandle("~");

      /// \brief setup custom callback queue
      gazebo_callback_queue_thread_ = new boost::thread( &GazeboRosApiPlugin::gazeboQueueThread,this );

      // below needs the world to be created first
      this->load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosApiPlugin::LoadGazeboRosApiPlugin,this,_1));
    }

    void GazeboRosApiPlugin::LoadGazeboRosApiPlugin(std::string _worldName)
    {
      // make sure things are only called once
      gazebo::event::Events::DisconnectWorldCreated(this->load_gazebo_ros_api_plugin_event_);
      this->lock_.lock();
      if (this->world_created_)
      {
        this->lock_.unlock();
        return;
      }

      // set flag to true and load this plugin
      this->world_created_ = true;
      this->lock_.unlock();


      this->world = gazebo::physics::get_world(_worldName);
      if (!this->world)
      {
        //ROS_ERROR("world name: [%s]",this->world->GetName().c_str());
        // connect helper function to signal for scheduling torque/forces, etc
        ROS_FATAL("cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
        return;
      }

      this->time_update_event_   = gazebo::event::Events::ConnectWorldUpdateStart(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));

      /// \brief advertise all services
      this->AdvertiseServices();

      // spin ros, is this needed?
      this->ros_spin_thread_ = new boost::thread(boost::bind(&GazeboRosApiPlugin::spin, this));

    }

    /// \brief ros queue thread for this node
    void GazeboRosApiPlugin::gazeboQueueThread()
    {
      ROS_DEBUG_STREAM("Callback thread id=" << boost::this_thread::get_id());
      static const double timeout = 0.001;
      while (this->rosnode_->ok())
        this->gazebo_queue_.callAvailable(ros::WallDuration(timeout));
    }

    /// \brief advertise services
    void GazeboRosApiPlugin::AdvertiseServices()
    {
      // publish clock for simulated ros time
      pub_clock_ = this->rosnode_->advertise<rosgraph_msgs::Clock>("/clock",10);


      // Advertise more services on the custom queue
      std::string reset_simulation_service_name("reset_simulation");
      ros::AdvertiseServiceOptions reset_simulation_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
          reset_simulation_service_name,boost::bind(&GazeboRosApiPlugin::resetSimulation,this,_1,_2),
          ros::VoidPtr(), &this->gazebo_queue_);
      reset_simulation_service_ = this->rosnode_->advertiseService(reset_simulation_aso);

      // Advertise more services on the custom queue
      std::string reset_world_service_name("reset_world");
      ros::AdvertiseServiceOptions reset_world_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
          reset_world_service_name,boost::bind(&GazeboRosApiPlugin::resetWorld,this,_1,_2),
          ros::VoidPtr(), &this->gazebo_queue_);
      reset_world_service_ = this->rosnode_->advertiseService(reset_world_aso);

      // Advertise more services on the custom queue
      std::string pause_physics_service_name("pause_physics");
      ros::AdvertiseServiceOptions pause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
          pause_physics_service_name,boost::bind(&GazeboRosApiPlugin::pausePhysics,this,_1,_2),
          ros::VoidPtr(), &this->gazebo_queue_);
      pause_physics_service_ = this->rosnode_->advertiseService(pause_physics_aso);

      // Advertise more services on the custom queue
      std::string unpause_physics_service_name("unpause_physics");
      ros::AdvertiseServiceOptions unpause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
          unpause_physics_service_name,boost::bind(&GazeboRosApiPlugin::unpausePhysics,this,_1,_2),
          ros::VoidPtr(), &this->gazebo_queue_);
      unpause_physics_service_ = this->rosnode_->advertiseService(unpause_physics_aso);

      // set param for use_sim_time if not set by user alread
      this->rosnode_->setParam("/use_sim_time", true);

      // todo: contemplate setting environment variable ROBOT=sim here???

    }


    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool GazeboRosApiPlugin::resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
    {
      this->world->Reset();
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool GazeboRosApiPlugin::resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
    {
      this->world->ResetEntities(gazebo::physics::Base::MODEL);
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
    {
      this->world->SetPaused(true);
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
    {
      this->world->SetPaused(false);
      return true;
    }


    void GazeboRosApiPlugin::spin()
    {
      // todo: make a wait loop that does not provide extra ros::spin()
      ros::Rate r(10);
      try
      {
      while(ros::ok())
      {
        ros::spinOnce();
          r.sleep();
      }
      }
      catch (...)
      {
        // ros seems to throw exceptions on Rate::sleep() during shutdown
      }
    }

    void GazeboRosApiPlugin::publishSimTime()
    {
      gazebo::common::Time currentTime = this->world->GetSimTime();
      rosgraph_msgs::Clock ros_time_;
      ros_time_.clock.fromSec(currentTime.Double());
      //  publish time to ros
      this->pub_clock_.publish(ros_time_);
    }

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}

