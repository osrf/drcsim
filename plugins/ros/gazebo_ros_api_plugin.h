#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

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

    /// \brief ros queue thread for this ros node
    void gazeboQueueThread();

    /// \brief advertise ros services
    void AdvertiseServices();

    void onLinkStatesConnect();
    void onModelStatesConnect();
    void onLinkStatesDisconnect();
    void onModelStatesDisconnect();

#ifdef GAZEBO_MSGS
    bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res);
    bool spawnGazeboModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief delete model in simulation
    /// \param[in] req DeleteModel request message
    /// \param[out] res DeleteModel response message
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req,gazebo_msgs::DeleteModel::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get state of model in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetModelState
    /// \param[out] res response in the form of gazebo_msgs::GetModelState
    bool getModelState(gazebo_msgs::GetModelState::Request &req,gazebo_msgs::GetModelState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get properties of model in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetModelProperties
    /// \param[out] res response in the form of gazebo_msgs::GetModelProperties
    bool getModelProperties(gazebo_msgs::GetModelProperties::Request &req,gazebo_msgs::GetModelProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get properties of simulation world
    /// \param[in] req request in the form of gazebo_msgs::GetWorldProperties
    /// \param[out] res response in the form of gazebo_msgs::GetWorldProperties
    bool getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,gazebo_msgs::GetWorldProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get joint properties in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetJointProperties
    /// \param[out] res response in the form of gazebo_msgs::GetJointProperties
    bool getJointProperties(gazebo_msgs::GetJointProperties::Request &req,gazebo_msgs::GetJointProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get link properties in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetLinkProperties
    /// \param[out] res response in the form of gazebo_msgs::GetLinkProperties
    bool getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,gazebo_msgs::GetLinkProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get link state in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetLinkState
    /// \param[out] res response in the form of gazebo_msgs::GetLinkState
    bool getLinkState(gazebo_msgs::GetLinkState::Request &req,gazebo_msgs::GetLinkState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set link state in simulation
    /// \param[in] req request in the form of gazebo_msgs::SetLinkState
    /// \param[out] res response in the form of gazebo_msgs::SetLinkState
    bool setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,gazebo_msgs::SetLinkProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set physics properties in simulation
    /// \param[in] req request in the form of gazebo_msgs::SetPhysicsProperties
    /// \param[out] res response in the form of gazebo_msgs::SetPhysicsProperties
    bool setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,gazebo_msgs::SetPhysicsProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief get physics properties in simulation
    /// \param[in] req request in the form of gazebo_msgs::GetPhysicsProperties
    /// \param[out] res response in the form of gazebo_msgs::GetPhysicsProperties
    bool getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,gazebo_msgs::GetPhysicsProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set joint properties in simulation
    /// \param[in] req request in the form of gazebo_msgs::SetJointProperties
    /// \param[out] res response in the form of gazebo_msgs::SetJointProperties
    bool setJointProperties(gazebo_msgs::SetJointProperties::Request &req,gazebo_msgs::SetJointProperties::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set model state in simulation
    /// \param[in] req request in the form of gazebo_msgs::SetModelState
    /// \param[out] res response in the form of gazebo_msgs::SetModelState
    bool setModelState(gazebo_msgs::SetModelState::Request &req,gazebo_msgs::SetModelState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set model state in simulation
    /// \param[in] model_satate desired model name and state specified in gazebo_msgs::ModelState
    void updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief apply joint effort
    /// \param[in] req request in the form of gazebo_msgs::ApplyJointEffort
    /// \param[out] res response in the form of gazebo_msgs::ApplyJointEffort
    bool applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,gazebo_msgs::ApplyJointEffort::Response &res);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Reset simulation
    /// \param[in] req std_srvs::Empty
    /// \param[out] res std_srvs::Empty
    bool resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Reset simulation
    /// \param[in] req std_srvs::Empty
    /// \param[out] res std_srvs::Empty
    bool resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Pause simulation
    /// \param[in] req std_srvs::Empty
    /// \param[out] res std_srvs::Empty
    bool pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief  Unpause simulation
    /// \param[in] req std_srvs::Empty
    /// \param[out] res std_srvs::Empty
    bool unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

#ifdef GAZEBO_MSGS
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief clear joint force application from queue
    /// \param[in] req service request from gazebo_msgs::JointRequest
    /// \param[out] res service response from gazebo_msgs::JointRequest
    bool clearJointForces(gazebo_msgs::JointRequest::Request &req,gazebo_msgs::JointRequest::Response &res);

    /// \brief clear joint force from queue
    /// \param[in] joint_name name of the joint to be removed from joint force queue
    bool clearJointForces(std::string joint_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief clear linkf orce application from queue
    /// \param[in] req service request from gazebo_msgs::BodyRequest
    /// \param[out] res service response from gazebo_msgs::BodyRequest
    bool clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,gazebo_msgs::BodyRequest::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief clear link force application from queue
    /// \param[in] body_name name of the link to be removed from link force queue
    bool clearBodyWrenches(std::string body_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief set model configuration via gazebo_msgs::SetModelConfiguration
    /// \param[in] req service request from gazebo_msgs::SetModelConfiguration
    /// \param[out] res service response from gazebo_msgs::SetModelConfiguration
    bool setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,gazebo_msgs::SetModelConfiguration::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Set link state in simulation via message
    /// \param[in] req service request from gazebo_msgs::SetLinkState
    /// \param[in] res service response from gazebo_msgs::SetLinkState
    bool setLinkState(gazebo_msgs::SetLinkState::Request &req,gazebo_msgs::SetLinkState::Response &res);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Set link state in simulation via ros topic
    /// \param[in] link_state A gazebo_msgs::LinkState message
    void updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Request to apply forces to links in simulation
    /// \param[in] req service call request
    /// \param[out] res service call response
    /// \return true to service call succeeds
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
    /// \brief apply forces to links in simulation
    void wrenchBodySchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief apply forces to joints in simulation
    void forceJointSchedulerSlot();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Sets ROS time to simulation time
    /// by calling ros::Time::setNow and publish ros topic /clock
    void updateRosSimTime();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief publish ros topic ~link_states
    void publishLinkStates();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief publish ros topic ~model_states
    void publishModelStates();

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief String XML declaration from XML string
    /// \param model_xml if XML string contains XML declaration, it will be stripped
    void stripXmlDeclaration(std::string &model_xml);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify the robot model pose in old Gazebo pre-1.0 XML for spawning in simulation
    /// \param gazebo_model_xml contains URDF of the model, the pose will be overwritten
    /// \param[in] initial_xyq desired model pose linear offset
    /// \param[in] initial_q desired model pose quaternion
    void updateGazeboXmlModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify robot name field in old Gazebo pre-1.0 XML
    /// \param[in] model_name desired naem of the model
    /// \param gazebo_model_xml SDF of the model, name will be overwritten
    void updateGazeboXmlName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify the robot model pose in SDF for spawning in simulation
    /// \param gazebo_model_xml contains URDF of the model, the pose will be overwritten
    /// \param[in] initial_xyq desired model pose linear offset
    /// \param[in] initial_q desired model pose quaternion
    void updateGazeboSDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify robot name in the gazebo SDF
    /// \param[in] model_name desired naem of the model
    /// \param gazebo_model_xml SDF of the model, name will be overwritten
    void updateGazeboSDFName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify the robot model pose in URDF for spawning in simulation
    /// \param gazebo_model_xml contains URDF of the model, the pose will be overwritten
    /// \param[in] initial_xyq desired model pose linear offset
    /// \param[in] initial_q desired model pose quaternion
    void updateURDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Modify the robot name field in URDF
    /// \param[in] model_name Name to be given to the URDF
    /// \param gazebo_model_xml TiXmlDocument containing the robot URDF, name will be overwritten
    void updateURDFName(TiXmlDocument &gazebo_model_xml, std::string model_name);

    ////////////////////////////////////////////////////////////////////////////////
    /// \brief  Go through the robot xml and update "robotNamespace" in plugins
    /// \param robot_xml TiXmlNode containing entire robot model
    void walkChildAddRobotNamespace(TiXmlNode* robot_xml);

    std::string robot_namespace_;

#ifdef GAZEBO_MSGS
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief Spawn model via service
    /// \param[in] gazebo_model_xml TiXmlDocument containing the model
    /// \param[in] model_name name of the model in gazebo
    /// \param[out] res SpawnModel service message response
    bool spawnAndConfirm(TiXmlDocument &gazebo_model_xml, std::string model_name, gazebo_msgs::SpawnModel::Response &res);
#endif
};
}
#endif
