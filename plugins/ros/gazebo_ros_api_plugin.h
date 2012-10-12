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

#ifdef GAZEBO_MSGS
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/BodyRequest.h"

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"

#include "gazebo_msgs/ApplyBodyWrench.h"

#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

#include "gazebo_msgs/SetJointProperties.h"

#include "gazebo_msgs/GetWorldProperties.h"

#include "gazebo_msgs/GetModelProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"

#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/ApplyJointEffort.h"

#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"

// Topics
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#endif

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// For model pose transform to set custom joint angles
#include <ros/ros.h>
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#ifdef GAZEBO_MSGS
#include <gazebo_msgs/SetModelConfiguration.h>
#endif
#include <boost/shared_ptr.hpp>

#undef USE_DYNAMIC_RECONFIGURE
#ifdef USE_DYNAMIC_RECONFIGURE
// For physics dynamics reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazebo/PhysicsConfig.h>
#ifdef GAZEBO_MSGS
#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
#endif
#endif

#include <boost/algorithm/string.hpp>

//#include <tf/transform_broadcaster.h>

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
    bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res);
    bool spawnGazeboModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief delete model given name
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req,gazebo_msgs::DeleteModel::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getModelState(gazebo_msgs::GetModelState::Request &req,gazebo_msgs::GetModelState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getModelProperties(gazebo_msgs::GetModelProperties::Request &req,gazebo_msgs::GetModelProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,gazebo_msgs::GetWorldProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getJointProperties(gazebo_msgs::GetJointProperties::Request &req,gazebo_msgs::GetJointProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,gazebo_msgs::GetLinkProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getLinkState(gazebo_msgs::GetLinkState::Request &req,gazebo_msgs::GetLinkState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,gazebo_msgs::SetLinkProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,gazebo_msgs::SetPhysicsProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,gazebo_msgs::GetPhysicsProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setJointProperties(gazebo_msgs::SetJointProperties::Request &req,gazebo_msgs::SetJointProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setModelState(gazebo_msgs::SetModelState::Request &req,gazebo_msgs::SetModelState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,gazebo_msgs::ApplyJointEffort::Response &res);
#endif

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

#ifdef GAZEBO_MSGS
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool clearJointForces(gazebo_msgs::JointRequest::Request &req,gazebo_msgs::JointRequest::Response &res);
    bool clearJointForces(std::string joint_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,gazebo_msgs::BodyRequest::Response &res);
    bool clearBodyWrenches(std::string body_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,gazebo_msgs::SetModelConfiguration::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool setLinkState(gazebo_msgs::SetLinkState::Request &req,gazebo_msgs::SetLinkState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,gazebo_msgs::ApplyBodyWrench::Response &res);
#endif

    void spin();

  private:
    // helper function for applyBodyWrench
    void transformWrench(gazebo::math::Vector3 &target_force, gazebo::math::Vector3 &target_torque,
                         gazebo::math::Vector3 reference_force, gazebo::math::Vector3 reference_torque,
                         gazebo::math::Pose target_to_reference );
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
    ros::Publisher     pub_link_states_;
    ros::Publisher     pub_model_states_;
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
    void PhysicsReconfigureCallback(gazebo::PhysicsConfig &config, uint32_t level);
    void PhysicsReconfigureNode();
#endif

    void OnResponse(ConstResponsePtr &_response);

    ros::Publisher     pub_clock_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private: boost::mutex lock_;

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
        double force; // should this be a array?
        ros::Time start_time;
        ros::Duration duration;
    };

    std::vector<GazeboRosApiPlugin::WrenchBodyJob*> wrench_body_jobs;
    std::vector<GazeboRosApiPlugin::ForceJointJob*> force_joint_jobs;

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void wrenchBodySchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void forceJointSchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void publishSimTime(const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &msg);
    void publishSimTime();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void publishLinkStates();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void publishModelStates();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void stripXmlDeclaration(std::string &model_xml);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateGazeboXmlModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateGazeboXmlName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateGazeboSDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateGazeboSDFName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateURDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void updateURDFName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    void walkChildAddRobotNamespace(TiXmlNode* robot_xml);

    std::string robot_namespace_;

#ifdef GAZEBO_MSGS
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief 
    bool spawnAndConfirm(TiXmlDocument &gazebo_model_xml, std::string model_name, gazebo_msgs::SpawnModel::Response &res);
#endif
};
}
#endif
