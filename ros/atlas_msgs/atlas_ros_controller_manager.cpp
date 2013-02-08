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
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <set>
#include <map>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <angles/angles.h>

#include "atlas_ros_controller_manager.h"

AtlasRosControllerManager::AtlasRosControllerManager()
{
  this->gotFirstJointStatesMsg = false;

  // must match those inside AtlasPlugin
  this->jointCommands.name.push_back("atlas::back_lbz");
  this->jointCommands.name.push_back("atlas::back_mby");
  this->jointCommands.name.push_back("atlas::back_ubx");
  this->jointCommands.name.push_back("atlas::neck_ay");
  this->jointCommands.name.push_back("atlas::l_leg_uhz");
  this->jointCommands.name.push_back("atlas::l_leg_mhx");
  this->jointCommands.name.push_back("atlas::l_leg_lhy");
  this->jointCommands.name.push_back("atlas::l_leg_kny");
  this->jointCommands.name.push_back("atlas::l_leg_uay");
  this->jointCommands.name.push_back("atlas::l_leg_lax");
  this->jointCommands.name.push_back("atlas::r_leg_uhz");
  this->jointCommands.name.push_back("atlas::r_leg_mhx");
  this->jointCommands.name.push_back("atlas::r_leg_lhy");
  this->jointCommands.name.push_back("atlas::r_leg_kny");
  this->jointCommands.name.push_back("atlas::r_leg_uay");
  this->jointCommands.name.push_back("atlas::r_leg_lax");
  this->jointCommands.name.push_back("atlas::l_arm_usy");
  this->jointCommands.name.push_back("atlas::l_arm_shx");
  this->jointCommands.name.push_back("atlas::l_arm_ely");
  this->jointCommands.name.push_back("atlas::l_arm_elx");
  this->jointCommands.name.push_back("atlas::l_arm_uwy");
  this->jointCommands.name.push_back("atlas::l_arm_mwx");
  this->jointCommands.name.push_back("atlas::r_arm_usy");
  this->jointCommands.name.push_back("atlas::r_arm_shx");
  this->jointCommands.name.push_back("atlas::r_arm_ely");
  this->jointCommands.name.push_back("atlas::r_arm_elx");
  this->jointCommands.name.push_back("atlas::r_arm_uwy");
  this->jointCommands.name.push_back("atlas::r_arm_mwx");

  unsigned int n = this->jointCommands.name.size();
  this->jointCommands.position.resize(n);
  this->jointCommands.velocity.resize(n);
  this->jointCommands.effort.resize(n);
  this->jointCommands.kp_position.resize(n);
  this->jointCommands.ki_position.resize(n);
  this->jointCommands.kd_position.resize(n);
  this->jointCommands.kp_velocity.resize(n);
  this->jointCommands.i_effort_min.resize(n);
  this->jointCommands.i_effort_max.resize(n);

}

AtlasRosControllerManager::~AtlasRosControllerManager()
{
  // If all previous steps are successful, start the controller manager
  // plugin updates
  delete this->controller_manager_;
  this->rosnode_->shutdown();

  this->controller_manager_queue_.clear();
  this->controller_manager_queue_.disable();
  this->controller_manager_callback_queue_thread_.join();

  // this->ros_spinner_thread_.join();

  delete this->rosnode_;

  if (this->virtual_mechanism_state_)
  {
    delete this->virtual_mechanism_state_;
  }
}

void AtlasRosControllerManager::Init(int argc, char** argv)
{
  // Get then name of the parent model
  this->robotNamespace = "";
  this->robotParam = "robot_description";
  this->robotParam = this->robotNamespace+"/" + this->robotParam;

  // Exit if no ROS
  ros::init(argc, argv, "atlas_ros_controller_manager");

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);
  ROS_INFO("starting atlas_ros_controller_manager plugin in ns: %s",
    this->robotNamespace.c_str());

  // pr2_etherCAT calls ros::spin(), we'll thread out one spinner
  // here to mimic that
  // this->ros_spinner_thread_ = boost::thread(
  //   boost::bind(&AtlasRosControllerManager::ControllerManagerROSThread, this));

  // Push commands to robot
  this->pubJointCommands =
    this->rosnode_->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  // set controller gains to 0, using effort only
  for (unsigned int i = 0; i < this->jointCommands.name.size(); ++i)
  {
    this->jointCommands.kp_position[i]  = 0;
    this->jointCommands.ki_position[i]  = 0;
    this->jointCommands.kd_position[i]  = 0;
    this->jointCommands.i_effort_min[i] = 0;
    this->jointCommands.i_effort_max[i] = 0;
    this->jointCommands.kp_velocity[i]  = 0;
    this->jointCommands.velocity[i]     = 0;
    this->jointCommands.effort[i]       = 0;
  }

  // load a controller manager, initialize hardware_interface
  this->controller_manager_ = new pr2_controller_manager::ControllerManager(
    &hardware_interface_, *this->rosnode_);

  // get sim time
  // this->hardware_interface_.current_time_ = ros::Time::now();

  // hardcoded to minimum of 1ms on start up
  // if (this->hardware_interface_.current_time_ < ros::Time(0.001))
    this->hardware_interface_.current_time_ == ros::Time(0.001);

  this->rosnode_->param("/start_robot_calibrated",
    this->calibration_status_, true);

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  if (!LoadControllerManagerFromURDF())
  {
    ROS_ERROR("Error parsing URDF in atlas controller manager plugin,"
              " plugin not active.\n");
    return;
  }

  // Initializes the fake state (for running the transmissions backwards).
  this->virtual_mechanism_state_ =
    new pr2_mechanism_model::RobotState(&this->controller_manager_->model_);

  // start custom queue for controller manager
  this->controller_manager_callback_queue_thread_ = boost::thread(boost::bind(
    &AtlasRosControllerManager::ControllerManagerQueueThread, this));

  // Start listening for joint_states from atlas.
  // do this last
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, 
    boost::bind(&AtlasRosControllerManager::UpdateControllerForces, this, _1),
    ros::VoidPtr(), &this->controller_manager_queue_);
    // ros::VoidPtr(), this->rosnode_->getCallbackQueue());
  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
  this->subJointStates = this->rosnode_->subscribe(jointStatesSo);

  // fill a map from controller_manager_->state_->joint_states_
  // to joint names for fast access
  for (unsigned int i = 0;
       i < this->virtual_mechanism_state_->joint_states_.size(); ++i)
  {
    std::string joint_name =
      this->virtual_mechanism_state_->joint_states_[i].joint_->name;

    this->virtualJointsMap[std::string("atlas::")+joint_name] =
      &this->virtual_mechanism_state_->joint_states_[i];
  }

}

////////////////////////////////////////////////////////////////////////////////
// custom callback queue
void AtlasRosControllerManager::ControllerManagerQueueThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->controller_manager_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void AtlasRosControllerManager::propagateSimulationToMechanismState(
  const sensor_msgs::JointState::ConstPtr &_jointStates)
{

  // FIXME
  // take values from received _jointStates push into
  // virtual_mechanism_state_->joint_states_
  for (unsigned int i = 0; i < _jointStates->name.size(); ++i)
  {
    pr2_mechanism_model::JointState* virtualJointState =
      this->virtualJointsMap.find(
      std::string("atlas::")+_jointStates->name[i])->second;

    if (virtualJointState != NULL)
    {
      // assuming commanded effort is exactly the measured effort.
      virtualJointState->measured_effort_ =
        virtualJointState->commanded_effort_;

      // copy position and velocity
      virtualJointState->position_ = virtualJointState->position_ +
        angles::shortest_angular_distance(
        virtualJointState->position_, _jointStates->position[i]);
      virtualJointState->velocity_ = _jointStates->velocity[i];
    }
    else
    {
      ROS_WARN("received joint state name [%s] not in mechanism states.",
        _jointStates->name[i].c_str());
    }
  }
}

void AtlasRosControllerManager::propagateMechanismStateForcesToSimulation()
{
  // FIXME
  // take forces virtual_mechanism_state_->joint_states_[i].commanded_effort_
  // and push them into jointCommands

  for (unsigned int i = 0; i < this->jointCommands.name.size(); ++i)
  {
    pr2_mechanism_model::JointState* virtualJointState =
      this->virtualJointsMap[this->jointCommands.name[i]];
    if (virtualJointState != NULL)
    {
      double effort = virtualJointState->commanded_effort_;
      this->jointCommands.effort[i] = effort;
    }
    else
    {
      ROS_WARN("robot jointCommands name [%s] not in robot joint states.",
        this->jointCommands.name[i].c_str());
    }
  }

  this->pubJointCommands.publish(this->jointCommands);
}

void AtlasRosControllerManager::UpdateControllerForces(
  const sensor_msgs::JointState::ConstPtr &_jointStates)
{
  if (!this->gotFirstJointStatesMsg)
  {
    this->gotFirstJointStatesMsg = true;
  }

  // for computing round trip time
  this->jointCommands.header.stamp = _jointStates->header.stamp;

  //--------------------------------------------------
  //  Update hardware time with sim time
  //--------------------------------------------------
  // this->hardware_interface_.current_time_ = ros::Time::now();
  this->hardware_interface_.current_time_ =
    ros::Time(_jointStates->header.stamp.sec, _jointStates->header.stamp.nsec);

  //--------------------------------------------------
  //  Pushes out simulation joint states into mechanism state
  //    1.  Set measured efforts to commanded effort
  //    2.  Simulation joint position --> mechanism joint states
  //    3.  Simulation joint velocity --> mechanism joint velocity
  //--------------------------------------------------
  this->propagateSimulationToMechanismState(_jointStates);

  //--------------------------------------------------
  //  Pushes mechanism states into actuator states
  //  Fill out the actuator state by reverse transmissions,
  //--------------------------------------------------
  this->virtual_mechanism_state_->propagateJointPositionToActuatorPosition();

  //--------------------------------------------------
  //  Update Mechanism Control
  //    with this update,
  //    mechanism state internally forward propagation from
  //    actuator state to its own copy of joint state
  //    use controllers to update joint force commands,
  //    then forward propagate from joint force to actuator forces
  //--------------------------------------------------
  try
  {
    this->controller_manager_->update();
  }
  catch(const char* c)
  {
    if (strcmp(c, "dividebyzero") == 0)
      ROS_WARN("pid controller reports divide by zero error");
    else
      ROS_WARN("unknown const char* exception: %s", c);
  }

  //--------------------------------------------------
  //  Propagate actuator efforts
  //    Propagate actuator forces to joint forces
  //--------------------------------------------------
  this->virtual_mechanism_state_->propagateActuatorEffortToJointEffort();


  //--------------------------------------------------
  //  Propagate joint state efforts to simulation
  //--------------------------------------------------
  this->propagateMechanismStateForcesToSimulation();

}


std::string AtlasRosControllerManager::GetURDF(std::string _param_name) const
{
  bool show_info_once = false;

  std::string urdf_string;
  urdf_string.clear();  // not really needed

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (this->rosnode_->searchParam(_param_name, search_param_name))
    {
      if (!show_info_once)
      {
        ROS_INFO("atlas ros controller manager plugin is waiting for model"
                 " URDF in parameter [%s] on the ROS param server.",
                 search_param_name.c_str());
        show_info_once = true;
      }
      this->rosnode_->getParam(search_param_name, urdf_string);
    }
    else
    {
      if (!show_info_once)
      {
        ROS_INFO("atlas ros controller manager plugin is waiting for model"
                 " URDF in parameter [%s] on the ROS param server.",
                 this->robotParam.c_str());
        show_info_once = true;
      }
      this->rosnode_->getParam(_param_name, urdf_string);
    }
    usleep(100000);
  }
  ROS_INFO("atlas ros controller manager parsing urdf from param server.");

  return urdf_string;
}

bool AtlasRosControllerManager::LoadControllerManagerFromURDF()
{
  std::string urdf_string = GetURDF(this->robotParam);

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()) && doc.Error())
  {
    ROS_ERROR("Could not load the atlas ros controller manager plugin's"
              " configuration file: %s\n", urdf_string.c_str());
    return false;
  }
  else
  {
    // debug
    // doc.Print();
    // std::cout << *(doc.RootElement()) << std::endl;

    // Pulls out the list of actuators used in the robot configuration.
    struct GetActuators : public TiXmlVisitor
    {
      std::set<std::string> actuators;
      virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
      {
        if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() ==
          std::string("rightActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() ==
          std::string("leftActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        return true;
      }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    // Places the found actuators into the hardware interface.
    std::set<std::string>::iterator it;
    for (it = get_actuators.actuators.begin();
         it != get_actuators.actuators.end(); ++it)
    {
      // std::cout << " adding actuator " << (*it) << std::endl;
      pr2_hardware_interface::Actuator* pr2_actuator =
        new pr2_hardware_interface::Actuator(*it);
      pr2_actuator->state_.is_enabled_ = true;
      this->hardware_interface_.addActuator(pr2_actuator);
    }

    // Setup mechanism control node
    this->controller_manager_->initXml(doc.RootElement());

    if (this->controller_manager_->state_ == NULL)
    {
      ROS_ERROR("Mechanism unable to parse robot_description URDF"
                " to fill out robot state in controller_manager.");
      return false;
    }

    // set fake calibration states for simulation
    for (unsigned int i = 0;
      i < this->controller_manager_->state_->joint_states_.size(); ++i)
      this->controller_manager_->state_->joint_states_[i].calibrated_ =
        calibration_status_;

    return true;
  }
}

// simulate the ros thread on the controller manager
// this ros thread will actually process any pending ros requests
void AtlasRosControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  while (this->rosnode_->ok())
  {
    usleep(1000);
    ros::spinOnce();
  }
}

