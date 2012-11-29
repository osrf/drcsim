/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gazebo_ros_controller_manager.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>

#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <common/common.hh>

#include <angles/angles.h>
#include <urdf/model.h>
#include <map>

namespace gazebo {

GazeboRosControllerManager::GazeboRosControllerManager()
{
}


GazeboRosControllerManager::~GazeboRosControllerManager()
{
  // If all previous steps are successful, start the controller manager plugin updates
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);

  delete this->controller_manager_; 
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->controller_manager_queue_.clear();
  this->controller_manager_queue_.disable();
  this->controller_manager_callback_queue_thread_.join();
#endif
  this->ros_spinner_thread_.join();

  delete this->rosnode_;

  if (this->virtual_mechanism_state_)
  {
    delete this->virtual_mechanism_state_;
  }
}

void GazeboRosControllerManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->parent_model_ = _parent;
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind( &GazeboRosControllerManager::LoadThread,this ) );
}

void GazeboRosControllerManager::LoadThread()
{
  // Get then name of the parent model
  std::string modelName = this->sdf->GetParent()->GetValueString("name");

  // Get the world name.
  this->world = this->parent_model_->GetWorld();

  // Error message if the model couldn't be found
  if (!this->parent_model_)
    ROS_ERROR("parent model in NULL.");



  if (getenv("CHECK_SPEEDUP"))
  {
    wall_start_ = this->world->GetRealTime().Double();
    sim_start_  = this->world->GetSimTime().Double();
  }

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  //if (this->updatePeriod > 0 &&
  //    (this->world->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
  //  ROS_ERROR("gazebo_ros_controller_manager update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");


  // get parameter name
  this->robotNamespace = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robotNamespace = this->sdf->GetElement("robotNamespace")->GetValueString();

  this->robotParam = "robot_description";
  if (this->sdf->HasElement("robotParam"))
    this->robotParam = this->sdf->GetElement("robotParam")->GetValueString();

  this->robotParam = this->robotNamespace+"/" + this->robotParam;

  // Init ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init( argc, argv, "gazebo", ros::init_options::NoSigintHandler);
    gzwarn << "should start ros::init in simulation by using the system plugin\n";
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);
  ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s",this->robotNamespace.c_str());

  // pr2_etherCAT calls ros::spin(), we'll thread out one spinner here to mimic that
  this->ros_spinner_thread_ = boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerROSThread,this ) );

  // load a controller manager, initialize hardware_interface
  this->controller_manager_ = new pr2_controller_manager::ControllerManager(&hardware_interface_,*this->rosnode_);
  this->hardware_interface_.current_time_ = ros::Time(this->world->GetSimTime().Double());
  // hardcoded to minimum of 1ms on start up
  if (this->hardware_interface_.current_time_ < ros::Time(0.001)) this->hardware_interface_.current_time_ == ros::Time(0.001);
  
  this->rosnode_->param("gazebo/start_robot_calibrated", this->calibration_status_, true);

  // read pr2 urdf
  // setup actuators, then setup mechanism control node
  if (!LoadControllerManagerFromURDF())
  {
    ROS_ERROR("Error parsing URDF in gazebo controller manager plugin, plugin not active.\n");
    return;
  }

  // Initializes the fake state (for running the transmissions backwards).
  this->virtual_mechanism_state_ = new pr2_mechanism_model::RobotState(&this->controller_manager_->model_);

  // The gazebo joints and mechanism joints should match up.
  for (unsigned int i = 0; i < this->controller_manager_->state_->joint_states_.size(); ++i)
  {
    std::string joint_name = this->controller_manager_->state_->joint_states_[i].joint_->name;

    // fill in gazebo joints pointer
    gazebo::physics::JointPtr joint = this->parent_model_->GetJoint(this->parent_model_->GetName()+"::"+joint_name);
    if (joint)
    {
      this->gazebo_joints_.push_back(joint);
    }
    else
    {
      ROS_WARN("A Mechanism Controlled joint named [%s] is not found in gazebo model[%s].\n",
        joint_name.c_str(), this->parent_model_->GetName().c_str());
      this->gazebo_joints_.push_back(gazebo::physics::JointPtr());
    }
  }
  assert(this->gazebo_joints_.size() == this->virtual_mechanism_state_->joint_states_.size());

#ifdef USE_CBQ
  // start custom queue for controller manager
  this->controller_manager_callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerQueueThread,this ) );
#endif

  // If all previous steps are successful, start the controller manager plugin updates
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosControllerManager::UpdateControllerForces, this));
}

void GazeboRosControllerManager::propagateSimulationToMechanismState()
{
  for (unsigned int i = 0; i < this->gazebo_joints_.size(); ++i)
  {
    if (!this->gazebo_joints_[i])
      continue;

    // assuming commanded effort is exactly the measured effort.
    this->virtual_mechanism_state_->joint_states_[i].measured_effort_ = this->virtual_mechanism_state_->joint_states_[i].commanded_effort_;

    // propagate gazebo joint states into virtual_mechanism_state_
    if (this->gazebo_joints_[i]->HasType(gazebo::physics::Base::HINGE_JOINT))
    {
      gazebo::physics::JointPtr hj = this->gazebo_joints_[i];
      this->virtual_mechanism_state_->joint_states_[i].position_ = this->virtual_mechanism_state_->joint_states_[i].position_ +
                    angles::shortest_angular_distance(this->virtual_mechanism_state_->joint_states_[i].position_,hj->GetAngle(0).Radian());
      this->virtual_mechanism_state_->joint_states_[i].velocity_ = hj->GetVelocity(0);
    }
    else if (this->gazebo_joints_[i]->HasType(gazebo::physics::Base::SLIDER_JOINT))
    {
      gazebo::physics::JointPtr sj = this->gazebo_joints_[i];
      {
        this->virtual_mechanism_state_->joint_states_[i].position_ = sj->GetAngle(0).Radian();
        this->virtual_mechanism_state_->joint_states_[i].velocity_ = sj->GetVelocity(0);
      }
    }
    else
    {
      ROS_DEBUG("this plugin only supports revolute and prismatic joints.");
    }
  }
}

void GazeboRosControllerManager::propagateMechanismStateForcesToSimulation()
{
  for (unsigned int i = 0; i < this->gazebo_joints_.size(); ++i)
  {
    if (!this->gazebo_joints_[i])
      continue;

    double effort = this->virtual_mechanism_state_->joint_states_[i].commanded_effort_;

    double damping_coef = 0;
    if (this->controller_manager_->state_ == NULL)
      ROS_WARN("Mechanism unable to parse robot_description URDF");
    else
    {
      if (this->controller_manager_->state_->joint_states_[i].joint_->dynamics)
        damping_coef = this->controller_manager_->state_->joint_states_[i].joint_->dynamics->damping;
    }

    if (this->gazebo_joints_[i]->HasType(gazebo::physics::Base::HINGE_JOINT))
    {
      gazebo::physics::JointPtr hj = this->gazebo_joints_[i];
      double current_velocity = hj->GetVelocity(0);
      double damping_force = 0.0 * damping_coef * current_velocity;
      double effort_command = effort - damping_force;
      hj->SetForce(0,effort_command);
    }
    else if (this->gazebo_joints_[i]->HasType(gazebo::physics::Base::SLIDER_JOINT))
    {
      gazebo::physics::JointPtr sj = this->gazebo_joints_[i];
      double current_velocity = sj->GetVelocity(0);
      double damping_force = 0.0 * damping_coef * current_velocity;
      double effort_command = effort-damping_force;
      sj->SetForce(0,effort_command);
    }
  }
}

void GazeboRosControllerManager::UpdateControllerForces()
{
  if (this->world->IsPaused()) return;

  if (getenv("CHECK_SPEEDUP"))
  {
    double wall_elapsed = this->world->GetRealTime().Double() - wall_start_;
    double sim_elapsed  = this->world->GetSimTime().Double()  - sim_start_;
    std::cout << " real time: " <<  wall_elapsed
              << "  sim time: " <<  sim_elapsed
              << "  speed up: " <<  sim_elapsed / wall_elapsed
              << std::endl;
  }

  //--------------------------------------------------
  //  Pushes out gazebo simulation state into mechanism state
  //    1.  Set measured efforts to commanded effort
  //    2.  Simulation joint position --> mechanism joint states
  //    3.  Simulation joint velocity --> mechanism joint velocity
  //--------------------------------------------------
  this->propagateSimulationToMechanismState();

  //--------------------------------------------------
  //  Pushes mechanism states into actuator states
  //  Fill out the actuator state by reverse transmissions,
  //--------------------------------------------------
  this->virtual_mechanism_state_->propagateJointPositionToActuatorPosition();

  //--------------------------------------------------
  //  Update hardware time with sim time
  //--------------------------------------------------
  this->hardware_interface_.current_time_ = ros::Time(this->world->GetSimTime().Double());
  //ROS_ERROR("time %d: %d", this->hardware_interface_.current_time_.sec, this->hardware_interface_.current_time_.nsec);

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
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
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


std::string GazeboRosControllerManager::GetURDF(std::string _param_name) const
{
  bool show_info_once = false;

  std::string urdf_string;
  urdf_string.clear(); // make sure it's empty

  // search and wait for robot_description on param server
  while(urdf_string.empty())
  {
    std::string search_param_name;
    if (this->rosnode_->searchParam(_param_name, search_param_name))
    {
      if (!show_info_once)
      {
        ROS_INFO("gazebo controller manager plugin is waiting for model URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());
        show_info_once = true;
      }
      this->rosnode_->getParam(search_param_name, urdf_string);
    }
    else
    {
      if (!show_info_once)
      {
        ROS_INFO("gazebo controller manager plugin is waiting for model URDF in parameter [%s] on the ROS param server.", this->robotParam.c_str());
        show_info_once = true;
      }
      this->rosnode_->getParam(_param_name, urdf_string);
    }
    usleep(100000);
  }
  ROS_INFO("gazebo controller manager got urdf from param server, parsing it...");

  return urdf_string;
}

bool GazeboRosControllerManager::LoadControllerManagerFromURDF()
{

  std::string urdf_string = GetURDF(this->robotParam);

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()) && doc.Error())
  {
    ROS_ERROR("Could not load the gazebo controller manager plugin's configuration file: %s\n",
            urdf_string.c_str());
    return false;
  }
  else
  {
    //doc.Print();
    //std::cout << *(doc.RootElement()) << std::endl;

    // Pulls out the list of actuators used in the robot configuration.
    struct GetActuators : public TiXmlVisitor
    {
      std::set<std::string> actuators;
      virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
      {
        if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        return true;
      }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    // Places the found actuators into the hardware interface.
    std::set<std::string>::iterator it;
    for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
    {
      //std::cout << " adding actuator " << (*it) << std::endl;
      pr2_hardware_interface::Actuator* pr2_actuator = new pr2_hardware_interface::Actuator(*it);
      pr2_actuator->state_.is_enabled_ = true;
      this->hardware_interface_.addActuator(pr2_actuator);
    }

    // Setup mechanism control node
    this->controller_manager_->initXml(doc.RootElement());

    if (this->controller_manager_->state_ == NULL)
    {
      ROS_ERROR("Mechanism unable to parse robot_description URDF to fill out robot state in controller_manager.");
      return false;
    }

    // set fake calibration states for simulation
    for (unsigned int i = 0; i < this->controller_manager_->state_->joint_states_.size(); ++i)
      this->controller_manager_->state_->joint_states_[i].calibrated_ = calibration_status_;

    return true;
  }
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// custom callback queue
void GazeboRosControllerManager::ControllerManagerQueueThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->controller_manager_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

// simulate the ros thread on the controller manager
// this ros thread will actually process any pending ros requests
void GazeboRosControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  while (this->rosnode_->ok())
  {
    usleep(1000);
    ros::spinOnce();
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosControllerManager)

} // namespace gazebo
