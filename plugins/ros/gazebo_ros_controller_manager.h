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

#ifndef GAZEBO_CONTROLLER_MANAGER_H
#define GAZEBO_CONTROLLER_MANAGER_H

#include <vector>
#include <map>

#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/physics.h"
#include "common/Time.hh"
#include "common/Plugin.hh"

#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_controller_manager/controller_manager.h"
#include "pr2_mechanism_model/robot.h"
#include <tinyxml.h>
#include <ros/ros.h>
#undef USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#endif

#include "boost/thread/mutex.hpp"

namespace gazebo
{

class GazeboRosControllerManager : public ModelPlugin
{
public:
  GazeboRosControllerManager();
  virtual ~GazeboRosControllerManager();
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

private:

  /// Callback state / effort propagations, controller updates at every simulation update time step
  /// There are 3 separate states at work here
  ///   - Simulation States (SS) from gazebo
  ///   - Virtual Mechanism State (VMS)
  ///   - Controller Manager's "Real" Robot Mechanism States (RMS)
  ///
  /// We introduce Virtual Mechanism State so we can exercise the actuator transmission
  /// used on the robot here in simulation.
  ///
  ///                                        +-----------------------------+
  /// SS:JointStates --> VMS:JointStates --> | Inverse State Transmissions | --> VMS:ActuatorStates
  ///                                        +-----------------------------+
  ///
  ///                                        +---------------------+
  /// VMS:ActuatorStates (or real robot) --> | State Transmissions | --> RMS:JointStates
  ///                                        +---------------------+
  ///
  ///                     +-------------+
  /// RMS:JointStates --> | Controllers | --> RMS:JointEfforts (this step is identical on the real robot)
  ///                     +-------------+
  ///
  ///                      +----------------------+
  /// RMS:JointEfforts --> | Effort Transmissions |  --> RMS:ActuatorEfforts (or real robot)
  ///                      +----------------------+
  ///
  ///                         +------------------------------+
  /// RMS:ActuatorEfforts --> | Inverse Effort Transmissions | --> VMS:JointEfforts --> SS:JointEfforts
  ///                         +------------------------------+
  ///
  void UpdateControllerForces();

  gazebo::physics::ModelPtr parent_model_;

  /// this interface holds actuator information as well as time
  pr2_hardware_interface::HardwareInterface hardware_interface_;

  /// this interface contains controllers and mechanism state
  pr2_controller_manager::ControllerManager *controller_manager_;

  /// we create this virtual mechanism state to obtain simulated actuator states using transmission
  pr2_mechanism_model::RobotState *virtual_mechanism_state_;

  /// A list of joints in simulation, this should match the list of joints in mechanism state 1-to-1
  std::vector<gazebo::physics::JointPtr>  gazebo_joints_;

  /// From URDF, fill out actuators in hardware interface,
  /// and initialize mechanism state within mechanism controller
  bool LoadControllerManagerFromURDF();
  std::string GetURDF(std::string _param_name);

  ///  Pushes out gazebo simulation state into mechanism state
  ///    1.  Set measured efforts to commanded effort
  ///    2.  Simulation joint position --> mechanism joint states
  ///    3.  Simulation joint velocity --> mechanism joint velocity
  void propagateSimulationToMechanismState();

  ///  Propagate joint state efforts to simulation
  ///    with some tweaks in efforts
  void propagateMechanismStateForcesToSimulation();

  /// \brief pointer to ros node
  ros::NodeHandle* rosnode_;

  /// \brief tmp vars for performance checking
  double wall_start_, sim_start_;

  /// \brief set topic name of robot description parameter
  //ParamT<std::string> *robotParamP;
  //ParamT<std::string> *robotNamespaceP;
  std::string robotParam;
  std::string robotNamespace;

  bool calibration_status_;

#ifdef USE_CBQ
  private: ros::CallbackQueue controller_manager_queue_;
  private: void ControllerManagerQueueThread();
  private: boost::thread controller_manager_callback_queue_thread_;
#endif
  private: void ControllerManagerROSThread();
  private: boost::thread ros_spinner_thread_;

  private: physics::WorldPtr world;

  private: event::ConnectionPtr updateConnection;
};

}

#endif

