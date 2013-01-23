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

/// skip gazebo_msgs usage for sake of dependency
#undef GAZEBO_MSGS

#include <boost/thread/mutex.hpp>

#include "gazebo/common/Events.hh"

#include "gazebo_ros_api_plugin.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)

  GazeboRosApiPlugin::GazeboRosApiPlugin()
  {
    this->robot_namespace_.clear();
    this->world_created_ = false;
  }

  GazeboRosApiPlugin::~GazeboRosApiPlugin()
  {
    // disconnect slots
    event::Events::DisconnectWorldUpdateStart(this->wrench_update_event_);
    event::Events::DisconnectWorldUpdateStart(this->force_update_event_);
    event::Events::DisconnectWorldUpdateStart(this->time_update_event_);

    if (this->pub_link_states_connection_count_ > 0)
      event::Events::DisconnectWorldUpdateStart(this->pub_link_states_event_);
    if (this->pub_model_states_connection_count_ > 0)
      event::Events::DisconnectWorldUpdateStart(this->pub_model_states_event_);

    // shutdown ros
    this->rosnode_->shutdown();
    delete this->rosnode_;

    // shutdown ros queue
    this->gazebo_callback_queue_thread_->join();
    delete this->gazebo_callback_queue_thread_;

#ifdef USE_DYNAMIC_RECONFIGURE
    this->physics_reconfigure_thread_->join();
    delete this->physics_reconfigure_thread_;
#endif

    this->ros_spin_thread_->join();
    delete this->ros_spin_thread_;

    /* Delete Force and Wrench Jobs */
    {
      boost::mutex::scoped_lock lock(this->lock_);
      for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator
        iter = this->force_joint_jobs.begin();
        iter != this->force_joint_jobs.end();)
      {
        delete (*iter);
        this->force_joint_jobs.erase(iter);
      }
      for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator
        iter = this->wrench_body_jobs.begin();
        iter != this->wrench_body_jobs.end();)
      {
        delete (*iter);
        this->wrench_body_jobs.erase(iter);
      }
    }
  }

  void GazeboRosApiPlugin::Load(int argc, char** argv)
  {
    // setup ros related
    if (!ros::isInitialized())
      ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler);
    else
      ROS_ERROR("Something other than this gazebo_ros_api plugin started"
                " ros::init(...), command line arguments may not be"
                " parsed properly.");

    this->rosnode_ = new ros::NodeHandle("~");

    // setup custom callback queue
    gazebo_callback_queue_thread_ = new boost::thread(
      &GazeboRosApiPlugin::gazeboQueueThread, this);

#ifdef USE_DYNAMIC_RECONFIGURE
    // start a thread for the physics dynamic reconfigure node
    this->physics_reconfigure_thread_ = new boost::thread(
      boost::bind(&GazeboRosApiPlugin::PhysicsReconfigureNode, this));
    this->physics_reconfigure_initialized_ = false;
#endif

    // below needs the world to be created first
    this->load_gazebo_ros_api_plugin_event_ =
      event::Events::ConnectWorldCreated(
        boost::bind(&GazeboRosApiPlugin::LoadGazeboRosApiPlugin, this, _1));
  }

  void GazeboRosApiPlugin::LoadGazeboRosApiPlugin(std::string _worldName)
  {
    // make sure things are only called once
    event::Events::DisconnectWorldCreated(
      this->load_gazebo_ros_api_plugin_event_);
    {
      boost::mutex::scoped_lock lock(this->lock_);
      if (this->world_created_)
      {
        return;
      }

      // set flag to true and load this plugin
      this->world_created_ = true;
    }


    this->world = physics::get_world(_worldName);
    if (!this->world)
    {
      // ROS_ERROR("world name: [%s]", this->world->GetName().c_str());
      // connect helper function to signal for scheduling torque/forces, etc
      ROS_FATAL("cannot load gazebo ros api server plugin,"
                " physics::get_world() fails to return world");
      return;
    }

    // advertise all services
    this->AdvertiseServices();

    // reset topic connection counts
    this->pub_link_states_connection_count_ = 0;
    this->pub_model_states_connection_count_ = 0;

    // hooks for applying forces, publishing simtime on /clock
    this->wrench_update_event_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot, this));
    this->force_update_event_  = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot, this));
    this->time_update_event_   = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosApiPlugin::publishSimTime, this));

    this->gazebonode_ = transport::NodePtr(new transport::Node());
    this->gazebonode_->Init(_worldName);
    /// \TODO: does not work in server plugin?
    // this->stat_sub_ = this->gazebonode_->Subscribe("~/world_stats",
    //   &GazeboRosApiPlugin::publishSimTime, this);
    this->factory_pub_ = this->gazebonode_->Advertise<msgs::Factory>(
      "~/factory");
    this->request_pub_ = this->gazebonode_->Advertise<msgs::Request>(
      "~/request");
    this->response_sub_ = this->gazebonode_->Subscribe(
      "~/response", &GazeboRosApiPlugin::OnResponse, this);

    // spin ros, is this needed?
    this->ros_spin_thread_ = new boost::thread(boost::bind(
      &GazeboRosApiPlugin::spin, this));
  }

  void GazeboRosApiPlugin::OnResponse(ConstResponsePtr &_response)
  {
  }

  // ros queue thread for this node
  void GazeboRosApiPlugin::gazeboQueueThread()
  {
    ROS_DEBUG_STREAM("Callback thread id=" << boost::this_thread::get_id());
    static const double timeout = 0.001;
    while (this->rosnode_->ok())
      this->gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }

  // advertise services
  void GazeboRosApiPlugin::AdvertiseServices()
  {
    // publish clock for simulated ros time
    pub_clock_ = this->rosnode_->advertise<rosgraph_msgs::Clock>("/clock", 10);

#ifdef GAZEBO_MSGS
    // Advertise spawn services on the custom queue
    std::string spawn_gazebo_model_service_name("spawn_gazebo_model");
    ros::AdvertiseServiceOptions spawn_gazebo_model_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
        spawn_gazebo_model_service_name, boost::bind(
          &GazeboRosApiPlugin::spawnGazeboModel, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    spawn_urdf_gazebo_service_ = this->rosnode_->advertiseService(
      spawn_gazebo_model_aso);

    // Advertise spawn services on the custom queue
    std::string spawn_urdf_model_service_name("spawn_urdf_model");
    ros::AdvertiseServiceOptions spawn_urdf_model_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
        spawn_urdf_model_service_name, boost::bind(
          &GazeboRosApiPlugin::spawnURDFModel, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    spawn_urdf_model_service_ = this->rosnode_->advertiseService(
      spawn_urdf_model_aso);

    // Advertise delete services on the custom queue
    std::string delete_model_service_name("delete_model");
    ros::AdvertiseServiceOptions delete_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
        delete_model_service_name, boost::bind(
          &GazeboRosApiPlugin::deleteModel, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    delete_model_service_ = this->rosnode_->advertiseService(delete_aso);

    // Advertise more services on the custom queue
    std::string get_model_properties_service_name("get_model_properties");
    ros::AdvertiseServiceOptions get_model_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
        get_model_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::getModelProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_model_properties_service_ = this->rosnode_->advertiseService(
      get_model_properties_aso);

    // Advertise more services on the custom queue
    std::string get_model_state_service_name("get_model_state");
    ros::AdvertiseServiceOptions get_model_state_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelState>(
        get_model_state_service_name, boost::bind(
          &GazeboRosApiPlugin::getModelState, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_model_state_service_ = this->rosnode_->advertiseService(
      get_model_state_aso);

    // Advertise more services on the custom queue
    std::string get_world_properties_service_name("get_world_properties");
    ros::AdvertiseServiceOptions get_world_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetWorldProperties>(
        get_world_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::getWorldProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_world_properties_service_ = this->rosnode_->advertiseService(
      get_world_properties_aso);

    // Advertise more services on the custom queue
    std::string get_joint_properties_service_name("get_joint_properties");
    ros::AdvertiseServiceOptions get_joint_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetJointProperties>(
        get_joint_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::getJointProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_joint_properties_service_ = this->rosnode_->advertiseService(
      get_joint_properties_aso);

    // Advertise more services on the custom queue
    std::string get_link_properties_service_name("get_link_properties");
    ros::AdvertiseServiceOptions get_link_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkProperties>(
        get_link_properties_service_name, boost::bind(
        &GazeboRosApiPlugin::getLinkProperties, this, _1, _2),
        ros::VoidPtr(), &this->gazebo_queue_);
    get_link_properties_service_ = this->rosnode_->advertiseService(
      get_link_properties_aso);

    // Advertise more services on the custom queue
    std::string get_link_state_service_name("get_link_state");
    ros::AdvertiseServiceOptions get_link_state_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkState>(
        get_link_state_service_name, boost::bind(
          &GazeboRosApiPlugin::getLinkState, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_link_state_service_ = this->rosnode_->advertiseService(
      get_link_state_aso);

    // Advertise more services on the custom queue
    std::string set_link_properties_service_name("set_link_properties");
    ros::AdvertiseServiceOptions set_link_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkProperties>(
        set_link_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::setLinkProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_link_properties_service_ = this->rosnode_->advertiseService(
      set_link_properties_aso);

    // Advertise more services on the custom queue
    std::string set_physics_properties_service_name("set_physics_properties");
    ros::AdvertiseServiceOptions set_physics_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPhysicsProperties>(
        set_physics_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::setPhysicsProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_physics_properties_service_ = this->rosnode_->advertiseService(
      set_physics_properties_aso);

    // Advertise more services on the custom queue
    std::string get_physics_properties_service_name("get_physics_properties");
    ros::AdvertiseServiceOptions get_physics_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
        get_physics_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::getPhysicsProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    get_physics_properties_service_ = this->rosnode_->advertiseService(
      get_physics_properties_aso);

    // Advertise more services on the custom queue
    std::string apply_body_wrench_service_name("apply_body_wrench");
    ros::AdvertiseServiceOptions apply_body_wrench_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyBodyWrench>(
        apply_body_wrench_service_name, boost::bind(
          &GazeboRosApiPlugin::applyBodyWrench, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    apply_body_wrench_service_ = this->rosnode_->advertiseService(
      apply_body_wrench_aso);

    // Advertise more services on the custom queue
    std::string set_model_state_service_name("set_model_state");
    ros::AdvertiseServiceOptions set_model_state_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelState>(
        set_model_state_service_name, boost::bind(
          &GazeboRosApiPlugin::setModelState, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_model_state_service_ = this->rosnode_->advertiseService(
      set_model_state_aso);

    // Advertise more services on the custom queue
    std::string apply_joint_effort_service_name("apply_joint_effort");
    ros::AdvertiseServiceOptions apply_joint_effort_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyJointEffort>(
        apply_joint_effort_service_name, boost::bind(
          &GazeboRosApiPlugin::applyJointEffort, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    apply_joint_effort_service_ = this->rosnode_->advertiseService(
      apply_joint_effort_aso);

    // Advertise more services on the custom queue
    std::string set_joint_properties_service_name("set_joint_properties");
    ros::AdvertiseServiceOptions set_joint_properties_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointProperties>(
        set_joint_properties_service_name, boost::bind(
          &GazeboRosApiPlugin::setJointProperties, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_joint_properties_service_ = this->rosnode_->advertiseService(
      set_joint_properties_aso);

    // Advertise more services on the custom queue
    std::string set_model_configuration_service_name("set_model_configuration");
    ros::AdvertiseServiceOptions set_model_configuration_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelConfiguration>(
        set_model_configuration_service_name, boost::bind(
          &GazeboRosApiPlugin::setModelConfiguration, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_model_configuration_service_ = this->rosnode_->advertiseService(
      set_model_configuration_aso);

    // Advertise more services on the custom queue
    std::string set_link_state_service_name("set_link_state");
    ros::AdvertiseServiceOptions set_link_state_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkState>(
        set_link_state_service_name, boost::bind(
          &GazeboRosApiPlugin::setLinkState, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    set_link_state_service_ = this->rosnode_->advertiseService(
      set_link_state_aso);
#endif

    // Advertise more services on the custom queue
    std::string reset_simulation_service_name("reset_simulation");
    ros::AdvertiseServiceOptions reset_simulation_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        reset_simulation_service_name, boost::bind(
          &GazeboRosApiPlugin::resetSimulation, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    reset_simulation_service_ = this->rosnode_->advertiseService(
      reset_simulation_aso);

    // Advertise more services on the custom queue
    std::string reset_world_service_name("reset_models");
    ros::AdvertiseServiceOptions reset_world_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        reset_world_service_name, boost::bind(
          &GazeboRosApiPlugin::resetWorld, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    reset_world_service_ = this->rosnode_->advertiseService(reset_world_aso);

    // Advertise more services on the custom queue
    std::string pause_physics_service_name("pause_physics");
    ros::AdvertiseServiceOptions pause_physics_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        pause_physics_service_name, boost::bind(
          &GazeboRosApiPlugin::pausePhysics, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    pause_physics_service_ = this->rosnode_->advertiseService(
      pause_physics_aso);

    // Advertise more services on the custom queue
    std::string unpause_physics_service_name("unpause_physics");
    ros::AdvertiseServiceOptions unpause_physics_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        unpause_physics_service_name, boost::bind(
          &GazeboRosApiPlugin::unpausePhysics, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    unpause_physics_service_ = this->rosnode_->advertiseService(
      unpause_physics_aso);

#ifdef GAZEBO_MSGS
    // Advertise more services on the custom queue
    std::string clear_joint_forces_service_name("clear_joint_forces");
    ros::AdvertiseServiceOptions clear_joint_forces_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::JointRequest>(
        clear_joint_forces_service_name, boost::bind(
          &GazeboRosApiPlugin::clearJointForces, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    clear_joint_forces_service_ = this->rosnode_->advertiseService(
      clear_joint_forces_aso);

    // Advertise more services on the custom queue
    std::string clear_body_wrenches_service_name("clear_body_wrenches");
    ros::AdvertiseServiceOptions clear_body_wrenches_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::BodyRequest>(
        clear_body_wrenches_service_name, boost::bind(
          &GazeboRosApiPlugin::clearBodyWrenches, this, _1, _2),
          ros::VoidPtr(), &this->gazebo_queue_);
    clear_body_wrenches_service_ = this->rosnode_->advertiseService(
      clear_body_wrenches_aso);

    // Advertise topic on custom queue
    // topic callback version for set_link_state
    ros::SubscribeOptions link_state_so =
      ros::SubscribeOptions::create<gazebo_msgs::LinkState>(
      "set_link_state", 10, boost::bind(
           &GazeboRosApiPlugin::updateLinkState, this, _1),
        ros::VoidPtr(), &this->gazebo_queue_);
    set_link_state_topic_ = this->rosnode_->subscribe(link_state_so);

    // topic callback version for set_model_state
    ros::SubscribeOptions model_state_so =
      ros::SubscribeOptions::create<gazebo_msgs::ModelState>(
      "set_model_state", 10, boost::bind(
           &GazeboRosApiPlugin::updateModelState, this, _1),
        ros::VoidPtr(), &this->gazebo_queue_);
    set_model_state_topic_ = this->rosnode_->subscribe(model_state_so);

    // publish complete link states in world frame
    ros::AdvertiseOptions pub_link_states_ao =
      ros::AdvertiseOptions::create<gazebo_msgs::LinkStates>(
        "link_states", 10,
        boost::bind(&GazeboRosApiPlugin::onLinkStatesConnect, this),
        boost::bind(&GazeboRosApiPlugin::onLinkStatesDisconnect, this),
        ros::VoidPtr(), &this->gazebo_queue_);
    pub_link_states_ = this->rosnode_->advertise(pub_link_states_ao);

    // publish complete model states in world frame
    ros::AdvertiseOptions pub_model_states_ao =
      ros::AdvertiseOptions::create<gazebo_msgs::ModelStates>(
        "model_states", 10,
        boost::bind(&GazeboRosApiPlugin::onModelStatesConnect, this),
        boost::bind(&GazeboRosApiPlugin::onModelStatesDisconnect, this),
        ros::VoidPtr(), &this->gazebo_queue_);
    pub_model_states_ = this->rosnode_->advertise(pub_model_states_ao);
#endif

    // set param for use_sim_time if not set by user alread
    this->rosnode_->setParam("/use_sim_time", true);

    // todo: contemplate setting environment variable ROBOT=sim here???
  }

#ifdef GAZEBO_MSGS
  void GazeboRosApiPlugin::onLinkStatesConnect()
  {
    this->pub_link_states_connection_count_++;
    if (this->pub_link_states_connection_count_ == 1)
      this->pub_link_states_event_   = event::Events::ConnectWorldUpdateStart(
        boost::bind(&GazeboRosApiPlugin::publishLinkStates, this));
  }
  void GazeboRosApiPlugin::onModelStatesConnect()
  {
    this->pub_model_states_connection_count_++;
    if (this->pub_model_states_connection_count_ == 1)
      this->pub_model_states_event_   = event::Events::ConnectWorldUpdateStart(
        boost::bind(&GazeboRosApiPlugin::publishModelStates, this));
  }
  void GazeboRosApiPlugin::onLinkStatesDisconnect()
  {
    this->pub_link_states_connection_count_--;
    if (this->pub_link_states_connection_count_ <= 0)
    {
      event::Events::DisconnectWorldUpdateStart(this->pub_link_states_event_);
      if (this->pub_link_states_connection_count_ < 0)
        ROS_ERROR("one too mandy disconnect from pub_link_states_"
                  " in gazebo_ros.cpp? something weird");
    }
  }
  void GazeboRosApiPlugin::onModelStatesDisconnect()
  {
    this->pub_model_states_connection_count_--;
    if (this->pub_model_states_connection_count_ <= 0)
    {
      event::Events::DisconnectWorldUpdateStart(this->pub_model_states_event_);
      if (this->pub_model_states_connection_count_ < 0)
        ROS_ERROR("one too mandy disconnect from pub_model_states_"
                  " in gazebo_ros.cpp? something weird");
    }
  }
#endif

#ifdef GAZEBO_MSGS
  bool GazeboRosApiPlugin::spawnURDFModel(
    gazebo_msgs::SpawnModel::Request &req,
    gazebo_msgs::SpawnModel::Response &res)
  {
    // get name space for the corresponding model plugins
    this->robot_namespace_ = req.robot_namespace;

    // incoming robot name
    std::string model_name = req.model_name;

    // incoming robot model string
    std::string model_xml = req.model_xml;

    if (!this->IsURDF(model_xml))
    {
      ROS_ERROR("SpawnModel: Failure - model format is not URDF.");
      res.success = false;
      res.status_message = "SpawnModel: Failure - model format is not URDF.";
      return false;
    }

    /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
    /// @todo: does tinyxml have functionality for this?
    /// @todo: should gazebo take care of the declaration?
    {
      std::string open_bracket("<?");
      std::string close_bracket("?>");
      size_t pos1 = model_xml.find(open_bracket, 0);
      size_t pos2 = model_xml.find(close_bracket, 0);
      if (pos1 != std::string::npos && pos2 != std::string::npos)
        model_xml.replace(pos1, pos2-pos1+2, std::string(""));
    }


    // Now, replace package://xxx with the full path to the package
    {
      std::string package_prefix("package://");
      size_t pos1 = model_xml.find(package_prefix, 0);
      while (pos1 != std::string::npos)
      {
        size_t pos2 = model_xml.find("/", pos1+10);
        ROS_DEBUG(" pos %d %d", static_cast<int>pos1, static_cast<int>pos2);
        if (pos2 == std::string::npos || pos1 >= pos2)
        {
          ROS_ERROR("malformed package name?");
          break;
        }

        std::string package_name = model_xml.substr(pos1+10, pos2-pos1-10);
        ROS_DEBUG("package name [%s]", package_name.c_str());
        std::string package_path = ros::package::getPath(package_name);
        if (package_path.empty())
        {
          ROS_FATAL("Package[%s] does not have a path", package_name.c_str());
          res.success = false;
          res.status_message = std::string("urdf reference package name"
                                           " does not exist: ")+package_name;
          return false;
        }
        ROS_DEBUG("package name [%s] has path [%s]",
          package_name.c_str(), package_path.c_str());

        model_xml.replace(pos1, (pos2-pos1), package_path);
        pos1 = model_xml.find(package_prefix, 0);
      }
    }
    // ROS_DEBUG("Model XML\n\n%s\n\n ", model_xml.c_str());

    req.model_xml = model_xml;

    return spawnGazeboModel(req, res);
  }

  bool GazeboRosApiPlugin::spawnGazeboModel(
    gazebo_msgs::SpawnModel::Request &req,
    gazebo_msgs::SpawnModel::Response &res)
  {
    // incoming robot name
    std::string model_name = req.model_name;

    // get name space for the corresponding model plugins
    this->robot_namespace_ = req.robot_namespace;

    // get initial pose of model
    math::Vector3 initial_xyz(req.initial_pose.position.x,
                              req.initial_pose.position.y,
                              req.initial_pose.position.z);
    // get initial roll pitch yaw (fixed frame transform)
    math::Quaternion initial_q(req.initial_pose.orientation.w,
                               req.initial_pose.orientation.x,
                               req.initial_pose.orientation.y,
                               req.initial_pose.orientation.z);

    // frame for initial pose definition, update initial pose if defined
    physics::LinkPtr frame = boost::shared_dynamic_cast<physics::Link>(
      this->world->GetEntity(req.reference_frame));
    if (frame)
    {
      // convert to relative pose
      math::Pose frame_pose = frame->GetWorldPose();
      initial_xyz = frame_pose.rot.RotateVector(initial_xyz);
      initial_xyz += frame_pose.pos;
      initial_q *= frame_pose.rot;
    }

    /// @todo: map is really wrong, need to use tf here somehow
    else if (req.reference_frame == "" || req.reference_frame == "world" ||
             req.reference_frame == "map" || req.reference_frame == "/map")
    {
        ROS_DEBUG("SpawnModel: reference_frame is empty/world/map,"
                  " using inertial frame");
    }
    else
    {
      res.success = false;
      res.status_message = "SpawnModel: reference reference_frame not"
                  " found, did you forget to scope the link by model name?";
      return false;
    }

    // incoming robot model string
    std::string model_xml = req.model_xml;

    // store resulting Gazebo Model XML to be sent to spawn queue
    // get incoming string containg either an URDF or a Gazebo Model XML
    // grab from parameter server if necessary
    // convert to Gazebo Model XML if necessary


    this->stripXmlDeclaration(model_xml);


    // put string in TiXmlDocument for manipulation
    TiXmlDocument gazebo_model_xml;
    gazebo_model_xml.Parse(model_xml.c_str());

    // optional model manipulations: update initial pose && replace model name
    if (this->IsGazeboModelXML(model_xml))
    {
      this->updateGazeboXmlModelPose(gazebo_model_xml, initial_xyz, initial_q);
      this->updateGazeboXmlName(gazebo_model_xml, model_name);
      /// @todo: if (!this->robot_namespace_.empty())
      /// @todo:   this->walkChildAddRobotNamespace(robot);
    }
    else if (IsSDF(model_xml))
    {
      this->updateGazeboSDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
      this->updateGazeboSDFName(gazebo_model_xml, model_name);
    }
    else if (IsURDF(model_xml))
    {
      this->updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
      this->updateURDFName(gazebo_model_xml, model_name);
    }
    else
    {
      ROS_ERROR("GazeboRosApiPlugin SpawnModel Failure:"
                " input xml format not recognized");
      res.success = false;
      res.status_message = std::string("GazeboRosApiPlugin SpawnModel"
        " Failure: input model_xml not Gazebo XML,"
        " or cannot be converted to Gazebo XML");
      return false;
    }

    // do spawning check if spawn worked, return response
    return this->spawnAndConform(gazebo_model_xml, model_name, res);
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::deleteModel(gazebo_msgs::DeleteModel::Request &req,
    gazebo_msgs::DeleteModel::Response &res)
  {
    // clear forces, etc for the body in question
    physics::ModelPtr model = this->world->GetModel(req.model_name);
    if (!model)
    {
      ROS_ERROR("DeleteModel: model [%s] does not exist",
        req.model_name.c_str());
      res.success = false;
      res.status_message = "DeleteModel: model does not exist";
      return false;
    }

    // delete wrench jobs on bodies
    for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
    {
      physics::LinkPtr body =
        boost::dynamic_pointer_cast<physics::Link>(model->GetChild(i));
      if (body)
      {
        // look for it in jobs, delete body wrench jobs
        this->clearBodyWrenches(body->GetScopedName());
      }
    }

    // delete force jobs on joints
    for (unsigned int i = 0; i < model->GetJointCount(); i++)
    {
      physics::JointPtr joint = model->GetJoint(i);
      // look for it in jobs, delete joint force jobs
      this->clearJointForces(joint->GetName());
    }

    // clear entity from selection @todo: clear links if selected individually
    event::Events::setSelectedEntity(req.model_name);
    // send delete model request
    msgs::Request *msg = msgs::CreateRequest("entity_delete", req.model_name);
    this->request_pub_->Publish(*msg, true);

    ros::Duration model_spawn_timeout(60.0);
    ros::Time timeout = ros::Time::now() + model_spawn_timeout;
    // wait and verify that model is deleted
    while (true)
    {
      if (ros::Time::now() > timeout)
      {
        res.success = false;
        res.status_message = std::string("DeleteModel: Model pushed to"
          " delete queue, but delete service timed out waiting for "
          "model to disappear from simulation");
        return false;
      }
      {
        // boost::recursive_mutex::scoped_lock lock(*this->world->GetMRMutex());
        if (!this->world->GetModel(req.model_name)) break;
      }
      ROS_DEBUG("Waiting for model deletion (%s)", req.model_name.c_str());
      usleep(1000);
    }

    // set result
    res.success = true;
    res.status_message = std::string("DeleteModel: successfully deleted model");
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getModelState(
    gazebo_msgs::GetModelState::Request &req,
    gazebo_msgs::GetModelState::Response &res)
  {
    physics::ModelPtr model = this->world->GetModel(req.model_name);
    physics::LinkPtr frame = boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.relative_entity_name));
    if (!model)
    {
      ROS_ERROR("GetModelState: model [%s] does not exist",
        req.model_name.c_str());
      res.success = false;
      res.status_message = "GetModelState: model does not exist";
      return false;
    }
    else
    {
      // get model pose
      math::Pose       model_pose = model->GetWorldPose();
      math::Vector3    model_pos = model_pose.pos;
      math::Quaternion model_rot = model_pose.rot;

      // get model twist
      math::Vector3 model_linear_vel  = model->GetWorldLinearVel();
      math::Vector3 model_angular_vel = model->GetWorldAngularVel();


      if (frame)
      {
        // convert to relative pose
        math::Pose frame_pose = frame->GetWorldPose();
        model_pos = model_pos - frame_pose.pos;
        model_pos = frame_pose.rot.RotateVectorReverse(model_pos);
        model_rot *= frame_pose.rot.GetInverse();

        // convert to relative rates
        math::Vector3 frame_vpos = frame->GetWorldLinearVel();
        math::Vector3 frame_veul = frame->GetWorldAngularVel();
        model_linear_vel = frame_pose.rot.RotateVector(
          model_linear_vel - frame_vpos);
        model_angular_vel = frame_pose.rot.RotateVector(
          model_angular_vel - frame_veul);
      }
      /// @todo: FIXME map is really wrong, need to use tf here somehow
      else if (req.relative_entity_name == "" ||
               req.relative_entity_name == "world" ||
               req.relative_entity_name == "map" ||
               req.relative_entity_name == "/map")
      {
          ROS_DEBUG("GetModelState: relative_entity_name is"
                    " empty/world/map, using inertial frame");
      }
      else
      {
        res.success = false;
        res.status_message = "GetModelState: reference relative_entity_name"
          " not found, did you forget to scope the body by model name?";
        return false;
      }

      // fill in response
      res.pose.position.x = model_pos.x;
      res.pose.position.y = model_pos.y;
      res.pose.position.z = model_pos.z;
      res.pose.orientation.w = model_rot.w;
      res.pose.orientation.x = model_rot.x;
      res.pose.orientation.y = model_rot.y;
      res.pose.orientation.z = model_rot.z;

      res.twist.linear.x = model_linear_vel.x;
      res.twist.linear.y = model_linear_vel.y;
      res.twist.linear.z = model_linear_vel.z;
      res.twist.angular.x = model_angular_vel.x;
      res.twist.angular.y = model_angular_vel.y;
      res.twist.angular.z = model_angular_vel.z;

      res.success = true;
      res.status_message = "GetModelState: got properties";
      return true;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getModelProperties(
    gazebo_msgs::GetModelProperties::Request &req,
    gazebo_msgs::GetModelProperties::Response &res)
  {
    physics::ModelPtr model = this->world->GetModel(req.model_name);
    if (!model)
    {
      ROS_ERROR("GetModelProperties: model [%s] does not exist",
        req.model_name.c_str());
      res.success = false;
      res.status_message = "GetModelProperties: model does not exist";
      return false;
    }
    else
    {
      // get model parent name
      physics::ModelPtr parent_model =
        boost::dynamic_pointer_cast<physics::Model>(model->GetParent());
      if (parent_model) res.parent_model_name = parent_model->GetName();

      // get list of child bodies, geoms
      res.body_names.clear();
      res.geom_names.clear();
      for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
      {
        physics::LinkPtr body =
          boost::dynamic_pointer_cast<physics::Link>(model->GetChild(i));
        if (body)
        {
          res.body_names.push_back(body->GetName());
          // get list of geoms
          for (unsigned int j = 0; j < body->GetChildCount() ; j++)
          {
            physics::CollisionPtr geom =
              boost::dynamic_pointer_cast<physics::Collision>(
                body->GetChild(j));
            if (geom)
              res.geom_names.push_back(geom->GetName());
          }
        }
      }

      // get list of joints
      res.joint_names.clear();
      unsigned int jc = model->GetJointCount();
      for (unsigned int i = 0; i < jc ; i++)
        res.joint_names.push_back(model->GetJoint(i)->GetName());

      // get children model names
      res.child_model_names.clear();
      for (unsigned int j = 0; j < model->GetChildCount(); j++)
      {
        physics::ModelPtr child_model =
          boost::dynamic_pointer_cast<physics::Model>(model->GetChild(j));
        if (child_model)
          res.child_model_names.push_back(child_model->GetName());
      }

      // is model static
      res.is_static = model->IsStatic();

      res.success = true;
      res.status_message = "GetModelProperties: got properties";
      return true;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getWorldProperties(
    gazebo_msgs::GetWorldProperties::Request &req,
    gazebo_msgs::GetWorldProperties::Response &res)
  {
    res.sim_time = this->world->GetSimTime().Double();
    res.model_names.clear();
    for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
      res.model_names.push_back(this->world->GetModel(i)->GetName());
    gzerr << "disablign rendering has not been implemented, rendering"
          << " is always enabled\n";
    res.rendering_enabled = true;
    res.success = true;
    res.status_message = "GetWorldProperties: got properties";
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getJointProperties(
    gazebo_msgs::GetJointProperties::Request &req,
    gazebo_msgs::GetJointProperties::Response &res)
  {
    physics::JointPtr joint;
    for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
    {
      joint = this->world->GetModel(i)->GetJoint(req.joint_name);
      if (joint) break;
    }

    if (joint)
    {
      res.success = false;
      res.status_message = "GetJointProperties: joint not found";
      return false;
    }
    else
    {
      /// @todo: FIXME
      res.type = res.REVOLUTE;

      res.damping.clear();  // to be added to gazebo
      // res.damping.push_back(joint->GetDamping(0));

      res.position.clear();  // use GetAngle(i)
      res.position.push_back(joint->GetAngle(0).Radian());

      res.rate.clear();  // use GetVelocity(i)
      res.rate.push_back(joint->GetVelocity(0));

      res.success = true;
      res.status_message = "GetJointProperties: got properties";
      return true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getLinkProperties(
    gazebo_msgs::GetLinkProperties::Request &req,
    gazebo_msgs::GetLinkProperties::Response &res)
  {
    physics::LinkPtr body =
      boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(req.link_name));
    if (body)
    {
      res.success = false;
      res.status_message = "GetLinkProperties: link not found,"
        " did you forget to scope the link by model name?";
      return false;
    }
    else
    {
      /// @todo: validate
      res.gravity_mode = body->GetGravityMode();

      res.mass = body->GetInertial()->GetMass();

      physics::InertialPtr inertia = body->GetInertial();
      res.ixx = inertia->GetIXX();
      res.iyy = inertia->GetIYY();
      res.izz = inertia->GetIZZ();
      res.ixy = inertia->GetIXY();
      res.ixz = inertia->GetIXZ();
      res.iyz = inertia->GetIYZ();

      math::Vector3 com = body->GetInertial()->GetCoG();
      res.com.position.x = com.x;
      res.com.position.y = com.y;
      res.com.position.z = com.z;
      res.com.orientation.x = 0;
      res.com.orientation.y = 0;
      res.com.orientation.z = 0;
      res.com.orientation.w = 1;

      res.success = true;
      res.status_message = "GetLinkProperties: got properties";
      return true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getLinkState(
    gazebo_msgs::GetLinkState::Request &req,
    gazebo_msgs::GetLinkState::Response &res)
  {
    physics::LinkPtr body =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.link_name));
    physics::LinkPtr frame =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.reference_frame));

    if (body)
    {
      res.success = false;
      res.status_message = "GetLinkState: link not found, did you "
                           "forget to scope the link by model name?";
      return false;
    }

    // get body pose
    math::Pose body_pose = body->GetWorldPose();
    // Get inertial rates
    math::Vector3 body_vpos = body->GetWorldLinearVel();
    math::Vector3 body_veul = body->GetWorldAngularVel();

    if (frame)
    {
      // convert to relative pose
      math::Pose frame_pose = frame->GetWorldPose();
      body_pose.pos = body_pose.pos - frame_pose.pos;
      body_pose.pos = frame_pose.rot.RotateVectorReverse(body_pose.pos);
      body_pose.rot *= frame_pose.rot.GetInverse();

      // convert to relative rates
      math::Vector3 frame_vpos = frame->GetWorldLinearVel();
      math::Vector3 frame_veul = frame->GetWorldAngularVel();
      body_vpos = frame_pose.rot.RotateVector(body_vpos - frame_vpos);
      body_veul = frame_pose.rot.RotateVector(body_veul - frame_veul);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.reference_frame == "" || req.reference_frame == "world" ||
             req.reference_frame == "map" || req.reference_frame == "/map")
    {
        ROS_DEBUG("GetLinkState: reference_frame is empty/world/map,"
                  " using inertial frame");
    }
    else
    {
      res.success = false;
      res.status_message = "GetLinkState: reference reference_frame not found,"
                           " did you forget to scope the link by model name?";
      return false;
    }

    res.link_state.link_name = req.link_name;
    res.link_state.pose.position.x = body_pose.pos.x;
    res.link_state.pose.position.y = body_pose.pos.y;
    res.link_state.pose.position.z = body_pose.pos.z;
    res.link_state.pose.orientation.x = body_pose.rot.x;
    res.link_state.pose.orientation.y = body_pose.rot.y;
    res.link_state.pose.orientation.z = body_pose.rot.z;
    res.link_state.pose.orientation.w = body_pose.rot.w;
    res.link_state.twist.linear.x = body_vpos.x;
    res.link_state.twist.linear.y = body_vpos.y;
    res.link_state.twist.linear.z = body_vpos.z;
    res.link_state.twist.angular.x = body_veul.x;
    res.link_state.twist.angular.y = body_veul.y;
    res.link_state.twist.angular.z = body_veul.x;
    res.link_state.reference_frame = req.reference_frame;

    res.success = true;
    res.status_message = "GetLinkState: got state";
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setLinkProperties(
    gazebo_msgs::SetLinkProperties::Request &req,
    gazebo_msgs::SetLinkProperties::Response &res)
  {
    physics::LinkPtr body =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.link_name));
    if (body)
    {
      res.success = false;
      res.status_message = "SetLinkProperties: link not found, did you"
                           " forget to scope the link by model name?";
      return false;
    }
    else
    {
      physics::InertialPtr mass = body->GetInertial();
      // @todo: FIXME: add inertia matrix rotation to Gazebo
      // mass.SetInertiaRotation(math::Quaternionion(req.com.orientation.w,
      // res.com.orientation.x, req.com.orientation.y req.com.orientation.z));
      mass->SetCoG(math::Vector3(
        req.com.position.x, req.com.position.y, req.com.position.z));
      mass->SetInertiaMatrix(
        req.ixx, req.iyy, req.izz, req.ixy, req.ixz, req.iyz);
      mass->SetMass(req.mass);
      body->SetGravityMode(req.gravity_mode);
      // @todo: mass change unverified
      res.success = true;
      res.status_message = "SetLinkProperties: properties set";
      return true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setPhysicsProperties(
    gazebo_msgs::SetPhysicsProperties::Request &req,
    gazebo_msgs::SetPhysicsProperties::Response &res)
  {
    // pause simulation if requested
    bool is_paused = this->world->IsPaused();
    this->world->SetPaused(true);

    // supported updates
    physics::PhysicsEnginePtr ode_pe = (this->world->GetPhysicsEngine());
    ode_pe->SetStepTime(req.time_step);
    ode_pe->SetUpdateRate(req.max_update_rate);
    ode_pe->SetGravity(math::Vector3(req.gravity.x,
      req.gravity.y, req.gravity.z));

    // stuff only works in ODE right now
    ode_pe->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
    ode_pe->SetSORPGSPreconIters(req.ode_config.sor_pgs_precon_iters);
    ode_pe->SetSORPGSIters(req.ode_config.sor_pgs_iters);
    ode_pe->SetSORPGSW(req.ode_config.sor_pgs_w);
    ode_pe->SetWorldCFM(req.ode_config.cfm);
    ode_pe->SetWorldERP(req.ode_config.erp);
    ode_pe->SetContactSurfaceLayer(req.ode_config.contact_surface_layer);
    ode_pe->SetContactMaxCorrectingVel(
      req.ode_config.contact_max_correcting_vel);
    ode_pe->SetMaxContacts(req.ode_config.max_contacts);

    this->world->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::getPhysicsProperties(
    gazebo_msgs::GetPhysicsProperties::Request &req,
    gazebo_msgs::GetPhysicsProperties::Response &res)
  {
    // supported updates
    res.time_step = this->world->GetPhysicsEngine()->GetStepTime();
    res.pause = this->world->IsPaused();
    res.max_update_rate = this->world->GetPhysicsEngine()->GetUpdateRate();
    math::Vector3 gravity = this->world->GetPhysicsEngine()->GetGravity();
    res.gravity.x = gravity.x;
    res.gravity.y = gravity.y;
    res.gravity.z = gravity.z;

    // stuff only works in ODE right now
    res.ode_config.auto_disable_bodies =
      this->world->GetPhysicsEngine()->GetAutoDisableFlag();
    res.ode_config.sor_pgs_precon_iters =
      this->world->GetPhysicsEngine()->GetSORPGSPreconIters();
    res.ode_config.sor_pgs_iters =
      this->world->GetPhysicsEngine()->GetSORPGSIters();
    res.ode_config.sor_pgs_w = this->world->GetPhysicsEngine()->GetSORPGSW();
    res.ode_config.contact_surface_layer =
      this->world->GetPhysicsEngine()->GetContactSurfaceLayer();
    res.ode_config.contact_max_correcting_vel =
      this->world->GetPhysicsEngine()->GetContactMaxCorrectingVel();
    res.ode_config.cfm = this->world->GetPhysicsEngine()->GetWorldCFM();
    res.ode_config.erp = this->world->GetPhysicsEngine()->GetWorldERP();
    res.ode_config.max_contacts =
      this->world->GetPhysicsEngine()->GetMaxContacts();

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties";
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setJointProperties(
    gazebo_msgs::SetJointProperties::Request &req,
    gazebo_msgs::SetJointProperties::Response &res)
  {
    /// @todo: current settings only allows for setting of 1DOF joints
    /// (e.g. HingeJoint and SliderJoint) correctly.
    physics::JointPtr joint;
    for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
    {
      joint = this->world->GetModel(i)->GetJoint(req.joint_name);
      if (joint) break;
    }

    if (joint)
    {
      res.success = false;
      res.status_message = "SetJointProperties: joint not found";
      return false;
    }
    else
    {
      for (unsigned int i = 0; i < req.ode_joint_config.damping.size(); ++i)
        joint->SetDamping(i, req.ode_joint_config.damping[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.hiStop.size(); ++i)
        joint->SetAttribute(physics::Joint::HI_STOP, i,
          req.ode_joint_config.hiStop[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.loStop.size(); ++i)
        joint->SetAttribute(physics::Joint::LO_STOP, i,
          req.ode_joint_config.loStop[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.erp.size(); ++i)
        joint->SetAttribute(physics::Joint::ERP, i,
          req.ode_joint_config.erp[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.cfm.size(); ++i)
        joint->SetAttribute(physics::Joint::CFM, i,
          req.ode_joint_config.cfm[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.stop_erp.size(); ++i)
        joint->SetAttribute(physics::Joint::STOP_ERP, i,
          req.ode_joint_config.stop_erp[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.stop_cfm.size(); ++i)
        joint->SetAttribute(physics::Joint::STOP_CFM, i,
          req.ode_joint_config.stop_cfm[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.fudge_factor.size();
           ++i)
        joint->SetAttribute(physics::Joint::FUDGE_FACTOR, i,
          req.ode_joint_config.fudge_factor[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.fmax.size(); ++i)
        joint->SetAttribute(physics::Joint::FMAX, i,
          req.ode_joint_config.fmax[i]);
      for (unsigned int i = 0; i < req.ode_joint_config.vel.size(); ++i)
        joint->SetAttribute(physics::Joint::VEL, i,
          req.ode_joint_config.vel[i]);

      res.success = true;
      res.status_message = "SetJointProperties: properties set";
      return true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setModelState(
    gazebo_msgs::SetModelState::Request &req,
    gazebo_msgs::SetModelState::Response &res)
  {
    math::Vector3 target_pos(req.model_state.pose.position.x,
      req.model_state.pose.position.y, req.model_state.pose.position.z);
    math::Quaternion target_rot(req.model_state.pose.orientation.w,
      req.model_state.pose.orientation.x, req.model_state.pose.orientation.y,
      req.model_state.pose.orientation.z);
    target_rot.Normalize();  // eliminates invalid rotation (0, 0, 0, 0)
    math::Pose target_pose(target_pos, target_rot);
    math::Vector3 target_pos_dot(req.model_state.twist.linear.x,
      req.model_state.twist.linear.y, req.model_state.twist.linear.z);
    math::Vector3 target_rot_dot(req.model_state.twist.angular.x,
      req.model_state.twist.angular.y, req.model_state.twist.angular.z);

    physics::ModelPtr model = this->world->GetModel(req.model_state.model_name);
    if (!model)
    {
      ROS_ERROR("Updating ModelState: model [%s] does not exist",
        req.model_state.model_name.c_str());
      res.success = false;
      res.status_message = "SetModelState: model does not exist";
      return false;
    }
    else
    {
      physics::LinkPtr relative_entity =
        boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(req.model_state.reference_frame));
      if (relative_entity)
      {
        math::Pose  frame_pose = relative_entity->GetWorldPose();
        math::Vector3 frame_pos = frame_pose.pos;
        math::Quaternion frame_rot = frame_pose.rot;

        target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
        target_pose.rot = frame_rot * target_pose.rot;
      }
      /// @todo: FIXME map is really wrong, need to use tf here somehow
      else if (req.model_state.reference_frame == "" ||
               req.model_state.reference_frame == "world" ||
               req.model_state.reference_frame == "map" ||
               req.model_state.reference_frame == "/map")
      {
        ROS_DEBUG("Updating ModelState: reference frame is empty/world/map,"
                  " usig inertial frame");
      }
      else
      {
        ROS_ERROR("Updating ModelState: for model[%s], specified reference"
                  " frame entity [%s] does not exist",
                  req.model_state.model_name.c_str(),
                  req.model_state.reference_frame.c_str());
        res.success = false;
        res.status_message = "SetModelState: specified reference frame"
                             " entity does not exist";
        return false;
      }

      bool is_paused = this->world->IsPaused();
      this->world->SetPaused(true);
      model->SetWorldPose(target_pose);
      this->world->SetPaused(is_paused);

      // set model velocity
      model->SetLinearVel(target_pos_dot);
      model->SetAngularVel(target_rot_dot);

      res.success = true;
      res.status_message = "SetModelState: set model state done";
      return true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateModelState(
    const gazebo_msgs::ModelState::ConstPtr& model_state)
  {
    gazebo_msgs::SetModelState::Response res;
    gazebo_msgs::SetModelState::Request req;
    req.model_state = *model_state;
    /*bool success =*/ this->setModelState(req, res);
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::applyJointEffort(
    gazebo_msgs::ApplyJointEffort::Request &req,
    gazebo_msgs::ApplyJointEffort::Response &res)
  {
    physics::JointPtr joint;
    for (unsigned int i = 0; i < this->world->GetModelCount(); i ++)
    {
      joint = this->world->GetModel(i)->GetJoint(req.joint_name);
      if (joint)
      {
        GazeboRosApiPlugin::ForceJointJob* fjj =
          new GazeboRosApiPlugin::ForceJointJob;
        fjj->joint = joint;
        fjj->force = req.effort;
        fjj->start_time = req.start_time;
        if (fjj->start_time < ros::Time(this->world->GetSimTime().Double()))
          fjj->start_time = ros::Time(this->world->GetSimTime().Double());
        fjj->duration = req.duration;
        {
          boost::mutex::scoped_lock lock(this->lock_);
          this->force_joint_jobs.push_back(fjj);
        }

        res.success = true;
        res.status_message = "ApplyJointEffort: effort set";
        return true;
      }
    }

    res.success = false;
    res.status_message = "ApplyJointEffort: joint not found";
    return false;
  }
#endif

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::resetSimulation(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    this->world->Reset();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::resetWorld(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
  {
    this->world->ResetEntities(physics::Base::MODEL);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
  {
    this->world->SetPaused(true);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
  {
    this->world->SetPaused(false);
    return true;
  }

#ifdef GAZEBO_MSGS
  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::clearJointForces(
    gazebo_msgs::JointRequest::Request &req,
    gazebo_msgs::JointRequest::Response &res)
  {
    return this->clearJointForces(req.joint_name);
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::clearJointForces(std::string joint_name)
  {
    bool search = true;
    boost::mutex::scoped_lock lock(this->lock_);
    while (search)
    {
      search = false;
      for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator
           iter = this->force_joint_jobs.begin();
           iter != this->force_joint_jobs.end(); ++iter)
      {
        if ((*iter)->joint->GetName() == joint_name)
        {
          // found one, search through again
          search = true;
          delete (*iter);
          this->force_joint_jobs.erase(iter);
          break;
        }
      }
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::clearBodyWrenches(
    gazebo_msgs::BodyRequest::Request &req,
    gazebo_msgs::BodyRequest::Response &res)
  {
    return this->clearBodyWrenches(req.body_name);
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::clearBodyWrenches(std::string body_name)
  {
    bool search = true;
    boost::mutex::scoped_lock lock(this->lock_);
    while (search)
    {
      search = false;
      for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator
           iter = this->wrench_body_jobs.begin();
           iter != this->wrench_body_jobs.end(); ++iter)
      {
        if ((*iter)->body->GetScopedName() == body_name)
        {
          // found one, search through again
          search = true;
          delete (*iter);
          this->wrench_body_jobs.erase(iter);
          break;
        }
      }
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setModelConfiguration(
    gazebo_msgs::SetModelConfiguration::Request &req,
    gazebo_msgs::SetModelConfiguration::Response &res)
  {
    std::string gazebo_model_name = req.model_name;

    // search for model with name
    physics::ModelPtr gazebo_model = this->world->GetModel(req.model_name);
    if (!gazebo_model)
    {
      ROS_ERROR("SetModelConfiguration: model [%s] does not exist",
        gazebo_model_name.c_str());
      res.success = false;
      res.status_message = "SetModelConfiguration: model does not exist";
      return false;
    }

    if (req.joint_names.size() == req.joint_positions.size())
    {
      std::map<std::string, double> joint_position_map;
      for (unsigned int i = 0; i < req.joint_names.size(); i++)
      {
        joint_position_map[req.joint_names[i]] = req.joint_positions[i];
      }

      // make the service call to pause gazebo
      bool is_paused = this->world->IsPaused();
      if (!is_paused) this->world->SetPaused(true);

      gazebo_model->SetJointPositions(joint_position_map);

      // resume paused state before this call
      this->world->SetPaused(is_paused);

      res.success = true;
      res.status_message = "SetModelConfiguration: success";
      return true;
    }
    else
    {
      res.success = false;
      res.status_message = "SetModelConfiguration: joint name and position"
                           " list have different lengths";
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::setLinkState(
    gazebo_msgs::SetLinkState::Request &req,
    gazebo_msgs::SetLinkState::Response &res)
  {
    physics::LinkPtr body =
      boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(req.link_state.link_name));
    physics::LinkPtr frame =
      boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(req.link_state.reference_frame));
    if (!body)
    {
      ROS_ERROR("Updating LinkState: link [%s] does not exist",
        req.link_state.link_name.c_str());
      res.success = false;
      res.status_message = "SetLinkState: link does not exist";
      return false;
    }

    /// @todo: FIXME map is really wrong, unless using tf here somehow
    // get reference frame (body/model(link)) pose and
    // transform target pose to absolute world frame
    math::Vector3 target_pos(req.link_state.pose.position.x,
                             req.link_state.pose.position.y,
                             req.link_state.pose.position.z);
    math::Quaternion target_rot(req.link_state.pose.orientation.w,
                                req.link_state.pose.orientation.x,
                                req.link_state.pose.orientation.y,
                                req.link_state.pose.orientation.z);
    math::Pose target_pose(target_pos, target_rot);
    math::Vector3 target_linear_vel(req.link_state.twist.linear.x,
                                    req.link_state.twist.linear.y,
                                    req.link_state.twist.linear.z);
    math::Vector3 target_angular_vel(req.link_state.twist.angular.x,
                                     req.link_state.twist.angular.y,
                                     req.link_state.twist.angular.z);

    if (frame)
    {
      math::Pose  frame_pose = frame->GetWorldPose();
      math::Vector3 frame_pos = frame_pose.pos;
      math::Quaternion frame_rot = frame_pose.rot;

      target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
      target_pose.rot = frame_rot * target_pose.rot;

      math::Vector3 frame_linear_vel = frame->GetWorldLinearVel();
      math::Vector3 frame_angular_vel = frame->GetWorldAngularVel();
      target_linear_vel -= frame_linear_vel;
      target_angular_vel -= frame_angular_vel;
    }
    else if (req.link_state.reference_frame == "" ||
             req.link_state.reference_frame == "world" ||
             req.link_state.reference_frame == "map" ||
             req.link_state.reference_frame == "/map")
    {
      ROS_INFO("Updating LinkState: reference_frame is empty/world/map,"
               " using inertial frame");
    }
    else
    {
      ROS_ERROR("Updating LinkState: reference_frame is not a valid link name");
      res.success = false;
      res.status_message = "SetLinkState: failed";
      return false;
    }

    bool is_paused = this->world->IsPaused();
    if (!is_paused)
      this->world->SetPaused(true);
    body->SetWorldPose(target_pose);
    this->world->SetPaused(is_paused);

    // set body velocity to desired twist
    body->SetLinearVel(target_linear_vel);
    body->SetAngularVel(target_angular_vel);

    res.success = true;
    res.status_message = "SetLinkState: success";
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateLinkState(
    const gazebo_msgs::LinkState::ConstPtr& link_state)
  {
    gazebo_msgs::SetLinkState::Request req;
    gazebo_msgs::SetLinkState::Response res;
    req.link_state = *link_state;
    /*bool success = */  this->setLinkState(req, res);
  }


  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::transformWrench(math::Vector3 &target_force,
    math::Vector3 &target_torque, math::Vector3 reference_force,
    math::Vector3 reference_torque, math::Pose target_to_reference)
  {
    // rotate force into target frame
    target_force = target_to_reference.rot.RotateVector(reference_force);
    // rotate torque into target frame
    target_torque = target_to_reference.rot.RotateVector(reference_torque);

    // target force is the refence force rotated by the target->reference
    // transform
    target_force = target_force;
    target_torque = target_torque + target_to_reference.pos.Cross(target_force);
  }

  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::applyBodyWrench(
    gazebo_msgs::ApplyBodyWrench::Request &req,
    gazebo_msgs::ApplyBodyWrench::Response &res)
  {
    physics::LinkPtr body =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.body_name));
    physics::LinkPtr frame =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(req.reference_frame));
    if (!body)
    {
      ROS_ERROR("ApplyBodyWrench: body [%s] does not exist",
        req.body_name.c_str());
      res.success = false;
      res.status_message = "ApplyBodyWrench: body does not exist";
      return false;
    }

    // target wrench
    math::Vector3 reference_force(req.wrench.force.x,
                                  req.wrench.force.y,
                                  req.wrench.force.z);
    math::Vector3 reference_torque(req.wrench.torque.x,
                                   req.wrench.torque.y,
                                   req.wrench.torque.z);
    math::Vector3 reference_point(req.reference_point.x,
                                  req.reference_point.y,
                                  req.reference_point.z);

    math::Vector3 target_force;
    math::Vector3 target_torque;

    /// shift wrench to body frame if a non-zero reference point is given
    /// @todo: to be more general, should we make the reference point
    /// a reference pose?
    reference_torque = reference_torque +
      reference_point.Cross(reference_force);

    /// @todo: FIXME map is really wrong, need to use tf here somehow
    if (frame)
    {
      // get reference frame (body/model(body)) pose and
      // transform target pose to absolute world frame
      // @todo: need to modify wrench (target force and torque by frame)
      //        transform wrench from reference_point in reference_frame
      //        into the reference frame of the body
      //        first, translate by reference point to the body frame
      math::Pose target_to_reference = frame->GetWorldPose() -
        body->GetWorldPose();
      ROS_DEBUG("reference frame for applied wrench: [%f %f %f"
                ", %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
                 body->GetWorldPose().pos.x,
                 body->GetWorldPose().pos.y,
                 body->GetWorldPose().pos.z,
                 body->GetWorldPose().rot.GetAsEuler().x,
                 body->GetWorldPose().rot.GetAsEuler().y,
                 body->GetWorldPose().rot.GetAsEuler().z,
                 frame->GetWorldPose().pos.x,
                 frame->GetWorldPose().pos.y,
                 frame->GetWorldPose().pos.z,
                 frame->GetWorldPose().rot.GetAsEuler().x,
                 frame->GetWorldPose().rot.GetAsEuler().y,
                 frame->GetWorldPose().rot.GetAsEuler().z,
                 target_to_reference.pos.x,
                 target_to_reference.pos.y,
                 target_to_reference.pos.z,
                 target_to_reference.rot.GetAsEuler().x,
                 target_to_reference.rot.GetAsEuler().y,
                 target_to_reference.rot.GetAsEuler().z);
      this->transformWrench(target_force, target_torque, reference_force,
        reference_torque, target_to_reference);
      ROS_ERROR("wrench defined as [%s]:[%f %f %f, %f %f %f] -->"
                " applied as [%s]:[%f %f %f, %f %f %f]",
                 frame->GetName().c_str(),
                 reference_force.x,
                 reference_force.y,
                 reference_force.z,
                 reference_torque.x,
                 reference_torque.y,
                 reference_torque.z,
                 body->GetName().c_str(),
                 target_force.x,
                 target_force.y,
                 target_force.z,
                 target_torque.x,
                 target_torque.y,
                 target_torque.z);
    }
    else if (req.reference_frame == "" ||
             req.reference_frame == "world" ||
             req.reference_frame == "map" ||
             req.reference_frame == "/map")
    {
      ROS_INFO("ApplyBodyWrench: reference_frame is empty/world/map,"
               " using inertial frame, transferring from body relative"
               " to inertial frame");
      // FIXME: transfer to inertial frame
      math::Pose target_to_reference = body->GetWorldPose();
      target_force = reference_force;
      target_torque = reference_torque;

    }
    else
    {
      ROS_ERROR("ApplyBodyWrench: reference_frame is not a valid link name");
      res.success = false;
      res.status_message = "ApplyBodyWrench: reference_frame not found";
      return false;
    }

    // apply wrench
    // schedule a job to do below at appropriate times:
    // body->SetForce(force)
    // body->SetTorque(torque)
    GazeboRosApiPlugin::WrenchBodyJob* wej =
      new GazeboRosApiPlugin::WrenchBodyJob;
    wej->body = body;
    wej->force = target_force;
    wej->torque = target_torque;
    wej->start_time = req.start_time;
    if (wej->start_time < ros::Time(this->world->GetSimTime().Double()))
      wej->start_time = ros::Time(this->world->GetSimTime().Double());
    wej->duration = req.duration;
    {
      boost::mutex::scoped_lock lock(this->lock_);
      this->wrench_body_jobs.push_back(wej);
    }

    res.success = true;
    res.status_message = "";
    return true;
  }
#endif


  void GazeboRosApiPlugin::spin()
  {
    // todo: make a wait loop that does not provide extra ros::spin()
    ros::Rate r(10);
    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  // utilites for checking incoming string URDF/XML/Param
  bool GazeboRosApiPlugin::IsURDF(std::string model_xml)
  {
    TiXmlDocument doc_in;
    doc_in.Parse(model_xml.c_str());
    if (doc_in.FirstChild("robot"))
      return true;
    else
      return false;
  }
  bool GazeboRosApiPlugin::IsGazeboModelXML(std::string model_xml)
  {
    // FIXME: very crude check
    TiXmlDocument doc_in;
    doc_in.Parse(model_xml.c_str());
    if (doc_in.FirstChild("model:physical")) // old gazebo xml
      return true;
    else
      return false;
  }
  bool GazeboRosApiPlugin::IsSDF(std::string model_xml)
  {
    // FIXME: very crude check
    TiXmlDocument doc_in;
    doc_in.Parse(model_xml.c_str());
    if (doc_in.FirstChild("gazebo")) // sdf
      return true;
    else
      return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::wrenchBodySchedulerSlot()
  {
    // MDMutex locks in case model is getting deleted, don't have to
    // do this if we delete jobs first
    // boost::recursive_mutex::scoped_lock lock(*this->world->GetMDMutex());
    boost::mutex::scoped_lock lock(this->lock_);
    for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator
         iter = this->wrench_body_jobs.begin();
         iter != this->wrench_body_jobs.end(); )
    {
      // check times and apply wrench if necessary
      if (ros::Time(this->world->GetSimTime().Double()) >= (*iter)->start_time)
        if (ros::Time(this->world->GetSimTime().Double()) <=
            (*iter)->start_time+(*iter)->duration ||
            (*iter)->duration.toSec() < 0.0)
        {
          if ((*iter)->body)
          {
            (*iter)->body->SetForce((*iter)->force);
            (*iter)->body->SetTorque((*iter)->torque);
          }
          else
            (*iter)->duration.fromSec(0.0);
        }

      if (ros::Time(this->world->GetSimTime().Double()) >
          (*iter)->start_time+(*iter)->duration &&
          (*iter)->duration.toSec() >= 0.0)
      {
        // remove from queue once expires
        delete (*iter);
        this->wrench_body_jobs.erase(iter);
      }
      else
        iter++;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::forceJointSchedulerSlot()
  {
    // MDMutex locks in case model is getting deleted, don't have to
    // do this if we delete jobs first
    // boost::recursive_mutex::scoped_lock lock(*this->world->GetMDMutex());
    boost::mutex::scoped_lock lock(this->lock_);
    for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator
         iter = this->force_joint_jobs.begin();
         iter != this->force_joint_jobs.end(); )
    {
      // check times and apply force if necessary
      if (ros::Time(this->world->GetSimTime().Double()) >= (*iter)->start_time)
        if (ros::Time(this->world->GetSimTime().Double()) <=
            (*iter)->start_time+(*iter)->duration ||
            (*iter)->duration.toSec() < 0.0)
          {
            if ((*iter)->joint)
              (*iter)->joint->SetForce(0,(*iter)->force);
            else
              (*iter)->duration.fromSec(0.0);
          }

      if (ros::Time(this->world->GetSimTime().Double()) >
          (*iter)->start_time+(*iter)->duration &&
          (*iter)->duration.toSec() >= 0.0)
      {
        // remove from queue once expires
        this->force_joint_jobs.erase(iter);
      }
      else
        iter++;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::publishSimTime(
    const boost::shared_ptr<msgs::WorldStatistics const> &msg)
  {
    ROS_ERROR("CLOCK2");
    common::Time currentTime = msgs::Convert( msg->sim_time());
    rosgraph_msgs::Clock ros_time_;
    ros_time_.clock.fromSec(currentTime.Double());
    //  publish time to ros
    this->pub_clock_.publish(ros_time_);
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::publishSimTime()
  {
    common::Time currentTime = this->world->GetSimTime();
    rosgraph_msgs::Clock ros_time_;
    ros_time_.clock.fromSec(currentTime.Double());
    //  publish time to ros
    this->pub_clock_.publish(ros_time_);
  }

#ifdef GAZEBO_MSGS
  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::publishLinkStates()
  {
    gazebo_msgs::LinkStates link_states;

    // fill link_states
    for (unsigned int i = 0; i < this->world->GetModelCount(); i ++)
    {
      physics::ModelPtr model = this->world->GetModel(i);

      for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
      {
        physics::LinkPtr body =
          boost::dynamic_pointer_cast<physics::Link>(model->GetChild(j));

        if (body)
        {
          link_states.name.push_back(body->GetScopedName());
          geometry_msgs::Pose pose;
          math::Pose  body_pose = body->GetWorldPose();
          math::Vector3 pos = body_pose.pos;
          math::Quaternion rot = body_pose.rot;
          pose.position.x = pos.x;
          pose.position.y = pos.y;
          pose.position.z = pos.z;
          pose.orientation.w = rot.w;
          pose.orientation.x = rot.x;
          pose.orientation.y = rot.y;
          pose.orientation.z = rot.z;
          link_states.pose.push_back(pose);
          math::Vector3 linear_vel  = body->GetWorldLinearVel();
          math::Vector3 angular_vel = body->GetWorldAngularVel();
          geometry_msgs::Twist twist;
          twist.linear.x = linear_vel.x;
          twist.linear.y = linear_vel.y;
          twist.linear.z = linear_vel.z;
          twist.angular.x = angular_vel.x;
          twist.angular.y = angular_vel.y;
          twist.angular.z = angular_vel.z;
          link_states.twist.push_back(twist);
        }
      }
    }

    this->pub_link_states_.publish(link_states);
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::publishModelStates()
  {
    gazebo_msgs::ModelStates model_states;

    // fill model_states
    for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
    {
      physics::ModelPtr model = this->world->GetModel(i);
      model_states.name.push_back(model->GetName());
      geometry_msgs::Pose pose;
      math::Pose  model_pose = model->GetWorldPose();
      math::Vector3 pos = model_pose.pos;
      math::Quaternion rot = model_pose.rot;
      pose.position.x = pos.x;
      pose.position.y = pos.y;
      pose.position.z = pos.z;
      pose.orientation.w = rot.w;
      pose.orientation.x = rot.x;
      pose.orientation.y = rot.y;
      pose.orientation.z = rot.z;
      model_states.pose.push_back(pose);
      math::Vector3 linear_vel  = model->GetWorldLinearVel();
      math::Vector3 angular_vel = model->GetWorldAngularVel();
      geometry_msgs::Twist twist;
      twist.linear.x = linear_vel.x;
      twist.linear.y = linear_vel.y;
      twist.linear.z = linear_vel.z;
      twist.angular.x = angular_vel.x;
      twist.angular.y = angular_vel.y;
      twist.angular.z = angular_vel.z;
      model_states.twist.push_back(twist);
    }
    this->pub_model_states_.publish(model_states);
  }

#ifdef USE_DYNAMIC_RECONFIGURE
  void GazeboRosApiPlugin::PhysicsReconfigureCallback(
    PhysicsConfig &config, uint32_t level)
  {
    if (!physics_reconfigure_initialized_)
    {
      gazebo_msgs::GetPhysicsProperties srv;
      this->physics_reconfigure_get_client_.call(srv);

      config.time_step                  = srv.response.time_step;
      config.max_update_rate            = srv.response.max_update_rate;
      config.gravity_x                  = srv.response.gravity.x;
      config.gravity_y                  = srv.response.gravity.y;
      config.gravity_z                  = srv.response.gravity.z;
      config.auto_disable_bodies        =
        srv.response.ode_config.auto_disable_bodies;
      config.sor_pgs_precon_iters       =
        srv.response.ode_config.sor_pgs_precon_iters;
      config.sor_pgs_iters              = srv.response.ode_config.sor_pgs_iters;
      config.sor_pgs_rms_error_tol      =
        srv.response.ode_config.sor_pgs_rms_error_tol;
      config.sor_pgs_w                  = srv.response.ode_config.sor_pgs_w;
      config.contact_surface_layer      =
        srv.response.ode_config.contact_surface_layer;
      config.contact_max_correcting_vel =
        srv.response.ode_config.contact_max_correcting_vel;
      config.cfm                        = srv.response.ode_config.cfm;
      config.erp                        = srv.response.ode_config.erp;
      config.max_contacts               = srv.response.ode_config.max_contacts;
      physics_reconfigure_initialized_ = true;
    }
    else
    {
      bool changed = false;
      gazebo_msgs::GetPhysicsProperties srv;
      this->physics_reconfigure_get_client_.call(srv);

      // check for changes
      if (config.time_step != srv.response.time_step)
        changed = true;
      if (config.max_update_rate != srv.response.max_update_rate)
        changed = true;
      if (config.gravity_x != srv.response.gravity.x)
        changed = true;
      if (config.gravity_y != srv.response.gravity.y)
        changed = true;
      if (config.gravity_z != srv.response.gravity.z)
        changed = true;
      if (config.auto_disable_bodies !=
        srv.response.ode_config.auto_disable_bodies)
        changed = true;
      if ((uint32_t)config.sor_pgs_precon_iters !=
        srv.response.ode_config.sor_pgs_precon_iters)
        changed = true;
      if ((uint32_t)config.sor_pgs_iters !=
        srv.response.ode_config.sor_pgs_iters)
        changed = true;
      if (config.sor_pgs_rms_error_tol !=
        srv.response.ode_config.sor_pgs_rms_error_tol)
        changed = true;
      if (config.sor_pgs_w != srv.response.ode_config.sor_pgs_w)
        changed = true;
      if (config.contact_surface_layer !=
        srv.response.ode_config.contact_surface_layer)
        changed = true;
      if (config.contact_max_correcting_vel !=
        srv.response.ode_config.contact_max_correcting_vel)
        changed = true;
      if (config.cfm != srv.response.ode_config.cfm)
        changed = true;
      if (config.erp != srv.response.ode_config.erp)
        changed = true;
      if ((uint32_t)config.max_contacts != srv.response.ode_config.max_contacts)
        changed = true;

      if (changed)
      {
        // pause simulation if requested
        gazebo_msgs::SetPhysicsProperties srv;
        srv.request.time_step = config.time_step;
        srv.request.max_update_rate = config.max_update_rate;
        srv.request.gravity.x = config.gravity_x;
        srv.request.gravity.y = config.gravity_y;
        srv.request.gravity.z = config.gravity_z;
        srv.request.ode_config.auto_disable_bodies = config.auto_disable_bodies;
        srv.request.ode_config.sor_pgs_precon_iters =
          config.sor_pgs_precon_iters;
        srv.request.ode_config.sor_pgs_iters = config.sor_pgs_iters;
        srv.request.ode_config.sor_pgs_rms_error_tol =
          config.sor_pgs_rms_error_tol;
        srv.request.ode_config.sor_pgs_w = config.sor_pgs_w;
        srv.request.ode_config.contact_surface_layer =
          config.contact_surface_layer;
        srv.request.ode_config.contact_max_correcting_vel =
          config.contact_max_correcting_vel;
        srv.request.ode_config.cfm = config.cfm;
        srv.request.ode_config.erp = config.erp;
        srv.request.ode_config.max_contacts = config.max_contacts;
        this->physics_reconfigure_set_client_.call(srv);
        ROS_INFO("physics dynamics reconfigure update complete");
      }
      ROS_INFO("physics dynamics reconfigure complete");
    }
  }

  void GazeboRosApiPlugin::PhysicsReconfigureNode()
  {
    ros::NodeHandle node_handle;
    this->physics_reconfigure_set_client_ =
      node_handle.serviceClient<gazebo_msgs::SetPhysicsProperties>(
      "/gazebo/set_physics_properties");
    this->physics_reconfigure_get_client_ =
      node_handle.serviceClient<gazebo_msgs::GetPhysicsProperties>(
      "/gazebo/get_physics_properties");
    this->physics_reconfigure_set_client_.waitForExistence();
    this->physics_reconfigure_get_client_.waitForExistence();

    // for dynamic reconfigure physics
    // for dynamic_reconfigure
    dynamic_reconfigure::Server<PhysicsConfig> physics_reconfigure_srv;
    dynamic_reconfigure::Server<PhysicsConfig>::CallbackType
      physics_reconfigure_f;

    physics_reconfigure_f =
      boost::bind(&GazeboRosApiPlugin::PhysicsReconfigureCallback,
      this, _1, _2);
    physics_reconfigure_srv.setCallback(physics_reconfigure_f);

    ROS_INFO("Starting to spin physics dynamic reconfigure node...");
    ros::Rate r(10);
    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }
#endif
#endif

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::stripXmlDeclaration(std::string &model_xml)
  {
    // incoming robot model string is a string containing a Gazebo Model XML
    /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
    /// @todo: does tinyxml have functionality for this?
    /// @todo: should gazebo take care of the declaration?
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    size_t pos1 = model_xml.find(open_bracket, 0);
    size_t pos2 = model_xml.find(close_bracket, 0);
    if (pos1 != std::string::npos && pos2 != std::string::npos)
      model_xml.replace(pos1, pos2-pos1+2, std::string(""));
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateGazeboXmlModelPose(
    TiXmlDocument &gazebo_model_xml, math::Vector3 initial_xyz,
    math::Quaternion initial_q)
  {
    TiXmlElement* model_tixml =
      gazebo_model_xml.FirstChildElement("model:physical");

    if (model_tixml)
    {
      // replace initial pose of robot
      // find first instance of xyz and rpy, replace with initial pose
      TiXmlElement* xyz_key = model_tixml->FirstChildElement("xyz");
      if (xyz_key)
        model_tixml->RemoveChild(xyz_key);
      TiXmlElement* rpy_key = model_tixml->FirstChildElement("rpy");
      if (rpy_key)
        model_tixml->RemoveChild(rpy_key);

      xyz_key = new TiXmlElement("xyz");
      rpy_key = new TiXmlElement("rpy");

      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << initial_xyz.x << " " << initial_xyz.y << " "
                 << initial_xyz.z;
      math::Vector3 initial_rpy = initial_q.GetAsEuler();
      rpy_stream << initial_rpy.x*180.0/M_PI << " "
                 << initial_rpy.y*180.0/M_PI << " "
                 << initial_rpy.z*180.0/M_PI;


      TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
      TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

      xyz_key->LinkEndChild(xyz_txt);
      rpy_key->LinkEndChild(rpy_txt);

      model_tixml->LinkEndChild(xyz_key);
      model_tixml->LinkEndChild(rpy_key);
    }
    else
      ROS_ERROR("could not find <gazebo> element in sdf, so new name not"
                " applied");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateGazeboXmlName(TiXmlDocument &gazebo_model_xml, std::string model_name)
  {
    TiXmlElement* model_tixml =
      gazebo_model_xml.FirstChildElement("model:physical");

    // replace model name if one is specified by the user
    if (model_tixml)
    {
      if (model_tixml->Attribute("name") != NULL)
      {
        // removing old model name
        model_tixml->RemoveAttribute("name");
      }
      // replace with user specified name
      model_tixml->SetAttribute("name", model_name);
    }
    else
      ROS_ERROR("could not find <gazebo> element in sdf, so new name"
                " not applied");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateGazeboSDFModelPose(
    TiXmlDocument &gazebo_model_xml, math::Vector3 initial_xyz,
    math::Quaternion initial_q)
  {
    TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("gazebo");
    if (gazebo_tixml)
    {
      TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
      if (model_tixml)
      {
        // replace initial pose of robot
        // find first instance of xyz and rpy, replace with initial pose
        TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

        if (!origin_key)
        {
          origin_key = new TiXmlElement("origin");
          model_tixml->LinkEndChild(origin_key);
        }

        std::ostringstream origin_stream;
        math::Vector3 initial_rpy = initial_q.GetAsEuler();
        origin_stream << initial_xyz.x << " " << initial_xyz.y
                      << " " << initial_xyz.z << " "
                      << initial_rpy.x << " " << initial_rpy.y
                      << " " << initial_rpy.z;

        origin_key->SetAttribute("pose", origin_stream.str());
      }
      else
        ROS_ERROR("could not find <model> element in sdf, so name and"
                  " initial position is not applied");
    }
    else
      ROS_ERROR("could not find <gazebo> element in sdf, so new name"
                " not applied");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateGazeboSDFName(TiXmlDocument &gazebo_model_xml,
    std::string model_name)
  {
    TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("gazebo");
    if (gazebo_tixml)
    {
      TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
      if (model_tixml)
      {
        if (model_tixml->Attribute("name") != NULL)
        {
          // removing old model name
          model_tixml->RemoveAttribute("name");
        }
        // replace with user specified name
        model_tixml->SetAttribute("name", model_name);
      }
      else
        ROS_ERROR("could not find <model> element in sdf, so name and"
                  " initial position is not applied");
    }
    else
      ROS_ERROR("could not find <gazebo> element in sdf, so new name"
                " not applied");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
    math::Vector3 initial_xyz, math::Quaternion initial_q)
  {
    TiXmlElement* model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
    if (model_tixml)
    {
      // replace initial pose of robot
      // find first instance of xyz and rpy, replace with initial pose
      TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

      if (!origin_key)
      {
        origin_key = new TiXmlElement("origin");
        model_tixml->LinkEndChild(origin_key);
      }

      if (origin_key->Attribute("xyz"))
        origin_key->RemoveAttribute("xyz");
      if (origin_key->Attribute("rpy"))
        origin_key->RemoveAttribute("rpy");

      std::ostringstream xyz_stream;
      xyz_stream << initial_xyz.x << " " << initial_xyz.y
                 << " " << initial_xyz.z;

      std::ostringstream rpy_stream;
      math::Vector3 initial_rpy = initial_q.GetAsEuler();
      rpy_stream << initial_rpy.x << " " << initial_rpy.y
                 << " " << initial_rpy.z;

      origin_key->SetAttribute("xyz", xyz_stream.str());
      origin_key->SetAttribute("rpy", rpy_stream.str());
    }
    else
      ROS_ERROR("could not find <model> element in sdf, so name and initial position is not applied");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::updateURDFName(TiXmlDocument &gazebo_model_xml,
    std::string model_name)
  {
    TiXmlElement* model_tixml = gazebo_model_xml.FirstChildElement("robot");
    // replace model name if one is specified by the user
    if (model_tixml)
    {
      if (model_tixml->Attribute("name") != NULL)
      {
        // removing old model name
        model_tixml->RemoveAttribute("name");
      }
      // replace with user specified name
      model_tixml->SetAttribute("name", model_name);
    }
    else
      ROS_ERROR("could not find <robot> element in URDF, name not replaced");
  }

  //////////////////////////////////////////////////////////////////////////////
  void GazeboRosApiPlugin::walkChildAddRobotNamespace(TiXmlNode* robot_xml)
  {
    TiXmlNode* child = 0;
    child = robot_xml->IterateChildren(child);
    while (child != NULL)
    {
      if (child->ValueStr().find(std::string("plugin")) == 0 &&
          child->ValueStr().find(std::string("plugin")) != std::string::npos)
      {
        ROS_DEBUG("recursively walking gazebo extension for %s --> %d",
          child->ValueStr().c_str(),
          (int)child->ValueStr().find(std::string("plugin")));
        if (child->FirstChildElement("robotNamespace") == NULL)
        {
          ROS_DEBUG("    adding robotNamespace for %s",
            child->ValueStr().c_str());
          TiXmlElement* child_elem =
            child->ToElement()->FirstChildElement("robotNamespace");
          while (child_elem)
          {
            child->ToElement()->RemoveChild(child_elem);
            child_elem =
              child->ToElement()->FirstChildElement("robotNamespace");
          }
          TiXmlElement* key = new TiXmlElement("robotNamespace");
          TiXmlText* val = new TiXmlText(this->robot_namespace_);
          key->LinkEndChild(val);
          child->ToElement()->LinkEndChild(key);
        }
        else
        {
          ROS_DEBUG("    robotNamespace already exists for %s",
            child->ValueStr().c_str());
        }
      }
      this->walkChildAddRobotNamespace(child);
      child = robot_xml->IterateChildren(child);
    }
  }

#ifdef GAZEBO_MSGS
  //////////////////////////////////////////////////////////////////////////////
  bool GazeboRosApiPlugin::spawnAndConfirm(TiXmlDocument &gazebo_model_xml, std::string model_name, gazebo_msgs::SpawnModel::Response &res)
  {
    // push to factory iface
    std::ostringstream stream;
    stream << gazebo_model_xml;
    std::string gazebo_model_xml_string = stream.str();
    ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ", gazebo_model_xml_string.c_str());

    // publish to factory topic
    msgs::Factory msg;
    msgs::Init(msg, "spawn_model");
    msg.set_sdf(gazebo_model_xml_string);

    /// \todo: should use entity_info or add lock to World::receiveMutex
    // looking for Model to see if it exists already
    msgs::Request *entity_info_msg =
      msgs::CreateRequest("entity_info", model_name);
    this->request_pub_->Publish(*entity_info_msg, true);
    /// \todo: should wait for response response_sub_, check to see that
    // if _msg->response == "nonexistant"

    physics::ModelPtr model = this->world->GetModel(model_name);
    if (model)
    {
      ROS_ERROR("SpawnModel: Failure - model name %s already exist.",
        model_name.c_str());
      res.success = false;
      res.status_message = "SpawnModel: Failure - model already exists.";
      return false;
    }

    // Publish the factory message
    this->factory_pub_->Publish(msg);

    /// \todo: should change publish to direct invocation World::LoadModel()
    /// and/or change the poll for Model existence to common::Events check.

    /// poll and wait, verify that the model is spawned within 60 seconds
    ros::Duration model_spawn_timeout(60.0);
    ros::Time timeout = ros::Time::now() + model_spawn_timeout;
    while (true)
    {
      if (ros::Time::now() > timeout)
      {
        res.success = false;
        res.status_message = std::string("SpawnModel: Model pushed to spawn"
          " queue, but spawn service timed out waiting for model "
          "to appear in simulation");
        return false;
      }
      {
        //boost::recursive_mutex::scoped_lock lock(*this->world->GetMRMutex());
        if (this->world->GetModel(model_name)) break;
      }
      ROS_DEBUG("Waiting for spawning model (%s)", model_name.c_str());
      usleep(1000);
    }

    // set result
    res.success = true;
    res.status_message = std::string("SpawnModel: successfully spawned model");
    return true;
  }
#endif
}
