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

#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <tinyxml.h>

#include <boost/algorithm/string.hpp>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>

// Services
#include <std_srvs/Empty.h>

#ifdef GAZEBO_MSGS
#include <gazebo_msgs/JointRequest.h>
#include <gazebo_msgs/BodyRequest.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <gazebo_msgs/ApplyBodyWrench.h>

#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <gazebo_msgs/SetJointProperties.h>

#include <gazebo_msgs/GetWorldProperties.h>

#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/ApplyJointEffort.h>

#include <gazebo_msgs/GetLinkProperties.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>

// Topics
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#endif

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// For model pose transform to set custom joint angles
#ifdef GAZEBO_MSGS
#include <gazebo_msgs/SetModelConfiguration.h>
#endif
#include <boost/shared_ptr.hpp>

#undef USE_DYNAMIC_RECONFIGURE
#ifdef USE_DYNAMIC_RECONFIGURE
// For physics dynamics reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazebo/PhysicsConfig.h>
#endif

// #include <tf/transform_broadcaster.h>

#include <gazebo/Server.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
// #include "msgs/MessageTypes.hh"  // implicitly included from CommonTypes.hh

#include "PubQueue.h"

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

#ifdef GAZEBO_MSGS
    bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,
      gazebo_msgs::SpawnModel::Response &res);
    bool spawnGazeboModel(gazebo_msgs::SpawnModel::Request &req,
      gazebo_msgs::SpawnModel::Response &res);

    /// \brief delete a Model in simulation given name
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req,
      gazebo_msgs::DeleteModel::Response &res);

    /// \brief get dynamical state of a Model given its name
    bool getModelState(gazebo_msgs::GetModelState::Request &req,
      gazebo_msgs::GetModelState::Response &res);

    /// \brief get dynamical properties of a Model given its name
    bool getModelProperties(gazebo_msgs::GetModelProperties::Request &req,
      gazebo_msgs::GetModelProperties::Response &res);

    /// \brief get World properties
    bool getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,
      gazebo_msgs::GetWorldProperties::Response &res);

    /// \brief get Joint properties given its name
    bool getJointProperties(gazebo_msgs::GetJointProperties::Request &req,
      gazebo_msgs::GetJointProperties::Response &res);

    /// \brief todo
    bool getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,
      gazebo_msgs::GetLinkProperties::Response &res);

    /// \brief todo
    bool getLinkState(gazebo_msgs::GetLinkState::Request &req,
      gazebo_msgs::GetLinkState::Response &res);

    /// \brief todo
    bool setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,
      gazebo_msgs::SetLinkProperties::Response &res);

    /// \brief todo
    bool setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,
      gazebo_msgs::SetPhysicsProperties::Response &res);

    /// \brief todo
    bool getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,
      gazebo_msgs::GetPhysicsProperties::Response &res);

    /// \brief todo
    bool setJointProperties(gazebo_msgs::SetJointProperties::Request &req,
      gazebo_msgs::SetJointProperties::Response &res);

    /// \brief todo
    bool setModelState(gazebo_msgs::SetModelState::Request &req,
      gazebo_msgs::SetModelState::Response &res);

    /// \brief todo
    void updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state);

    /// \brief todo
    bool applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,
      gazebo_msgs::ApplyJointEffort::Response &res);
#endif

    /// \brief todo
    bool resetSimulation(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    /// \brief todo
    bool resetWorld(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    /// \brief todo
    bool pausePhysics(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    /// \brief todo
    bool unpausePhysics(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

#ifdef GAZEBO_MSGS
    /// \brief todo
    bool clearJointForces(gazebo_msgs::JointRequest::Request &req,
      gazebo_msgs::JointRequest::Response &res);
    bool clearJointForces(std::string joint_name);

    /// \brief todo
    bool clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,
      gazebo_msgs::BodyRequest::Response &res);
    bool clearBodyWrenches(std::string body_name);

    /// \brief todo
    bool setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,
      gazebo_msgs::SetModelConfiguration::Response &res);

    /// \brief todo
    bool setLinkState(gazebo_msgs::SetLinkState::Request &req,
      gazebo_msgs::SetLinkState::Response &res);

    /// \brief todo
    void updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state);

    /// \brief todo
    bool applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,
      gazebo_msgs::ApplyBodyWrench::Response &res);
#endif

    void spin();

  private:
    /// \brief helper function for applyBodyWrench
    /// shift wrench from reference frame to target frame
    /// assume wrench is defined
    void transformWrench(gazebo::math::Vector3 &target_force,
       gazebo::math::Vector3 &target_torque,
       gazebo::math::Vector3 reference_force,
       gazebo::math::Vector3 reference_torque,
       gazebo::math::Pose target_to_reference);

    gazebo::transport::NodePtr gazebonode_;
    gazebo::transport::SubscriberPtr stat_sub_;
    gazebo::transport::PublisherPtr factory_pub_;
    gazebo::transport::PublisherPtr request_pub_;
    gazebo::transport::SubscriberPtr response_sub_;

    ros::NodeHandle* rosnode_;
    ros::CallbackQueue gazebo_queue_;
    boost::thread* gazebo_callback_queue_thread_;

    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr wrench_update_event_;
    gazebo::event::ConnectionPtr force_update_event_;
    gazebo::event::ConnectionPtr time_update_event_;
    gazebo::event::ConnectionPtr pub_link_states_event_;
    gazebo::event::ConnectionPtr pub_model_states_event_;
    gazebo::event::ConnectionPtr load_gazebo_ros_api_plugin_event_;

    ros::ServiceServer spawn_urdf_gazebo_service_;
    ros::ServiceServer spawn_urdf_model_service_;
    ros::ServiceServer delete_model_service_;
    ros::ServiceServer get_model_state_service_;
    ros::ServiceServer get_model_properties_service_;
    ros::ServiceServer get_world_properties_service_;
    ros::ServiceServer get_joint_properties_service_;
    ros::ServiceServer get_link_properties_service_;
    ros::ServiceServer get_link_state_service_;
    ros::ServiceServer set_link_properties_service_;
    ros::ServiceServer set_physics_properties_service_;
    ros::ServiceServer get_physics_properties_service_;
    ros::ServiceServer apply_body_wrench_service_;
    ros::ServiceServer set_joint_properties_service_;
    ros::ServiceServer set_model_state_service_;
    ros::ServiceServer apply_joint_effort_service_;
    ros::ServiceServer set_model_configuration_service_;
    ros::ServiceServer set_link_state_service_;
    ros::ServiceServer reset_simulation_service_;
    ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;
    ros::ServiceServer clear_joint_forces_service_;
    ros::ServiceServer clear_body_wrenches_service_;
    ros::Subscriber    set_link_state_topic_;
    ros::Subscriber    set_model_state_topic_;
#ifdef GAZEBO_MSGS
    ros::Publisher     pub_link_states_;
    PubQueue<gazebo_msgs::LinkStates>::Ptr pub_link_states_queue_;
    ros::Publisher     pub_model_states_;
    PubQueue<gazebo_msgs::ModelStates>::Ptr pub_model_states_queue_;
#endif
    int                pub_link_states_connection_count_;
    int                pub_model_states_connection_count_;

    // for internal gazebo xml use
    std::string xmlPrefix_;
    std::string xmlSuffix_;

    boost::thread* ros_spin_thread_;

#ifdef USE_DYNAMIC_RECONFIGURE
    // physics dynamic reconfigure
    boost::thread* physics_reconfigure_thread_;
    bool physics_reconfigure_initialized_;
    ros::ServiceClient physics_reconfigure_set_client_;
    ros::ServiceClient physics_reconfigure_get_client_;
    void PhysicsReconfigureCallback(gazebo::PhysicsConfig &config,
      uint32_t level);
    void PhysicsReconfigureNode();
#endif

    void OnResponse(ConstResponsePtr &_response);

    ros::Publisher     pub_clock_;
    PubQueue<rosgraph_msgs::Clock>::Ptr pub_clock_queue_;

    /// \brief A mutex to lock access to fields that are used in
    /// ROS message callbacks
    boost::mutex lock_;

    /// \brief utilites for checking incoming string URDF/XML/Param
    bool IsURDF(std::string model_xml);
    bool IsGazeboModelXML(std::string model_xml);
    bool IsSDF(std::string model_xml);
    void LoadGazeboRosApiPlugin(std::string _worldName);
    bool world_created_;

    class WrenchBodyJob
    {
      public:
        gazebo::physics::LinkPtr body;
        gazebo::math::Vector3 force;
        gazebo::math::Vector3 torque;
        ros::Time start_time;
        ros::Duration duration;
    };

    class ForceJointJob
    {
      public:
        gazebo::physics::JointPtr joint;
        double force;
        ros::Time start_time;
        ros::Duration duration;
    };

    std::vector<GazeboRosApiPlugin::WrenchBodyJob*> wrench_body_jobs;
    std::vector<GazeboRosApiPlugin::ForceJointJob*> force_joint_jobs;

    /// \brief todo
    void wrenchBodySchedulerSlot();

    /// \brief todo
    void forceJointSchedulerSlot();

    /// \brief todo
    void publishSimTime(
      const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &msg);

    void publishSimTime();

    /// \brief todo
    void publishLinkStates();

    /// \brief todo
    void publishModelStates();

    /// \brief todo
    void stripXmlDeclaration(std::string &model_xml);

    /// \brief todo
    void updateGazeboXmlModelPose(TiXmlDocument &gazebo_model_xml,
      gazebo::math::Vector3 initial_xyz,
      gazebo::math::Quaternion initial_q);

    /// \brief todo
    void updateGazeboXmlName(TiXmlDocument &gazebo_model_xml,
      std::string model_name);

    /// \brief todo
    void updateGazeboSDFModelPose(TiXmlDocument &gazebo_model_xml,
      gazebo::math::Vector3 initial_xyz,
      gazebo::math::Quaternion initial_q);

    /// \brief todo
    void updateGazeboSDFName(TiXmlDocument &gazebo_model_xml,
      std::string model_name);

    /// \brief todo
    void updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
      gazebo::math::Vector3 initial_xyz,
      gazebo::math::Quaternion initial_q);

    /// \brief todo
    void updateURDFName(TiXmlDocument &gazebo_model_xml,
      std::string model_name);

    /// \brief todo
    void walkChildAddRobotNamespace(TiXmlNode* robot_xml);

    std::string robot_namespace_;

#ifdef GAZEBO_MSGS
    /// \brief todo
    bool spawnAndConfirm(TiXmlDocument &gazebo_model_xml,
      std::string model_name, gazebo_msgs::SpawnModel::Response &res);
#endif

    /// \brief ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
};
}
#endif
