/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include "AtlasControlTypes.h"
#include "AtlasSimInterface.h"
#include "AtlasSimInterfaceTypes.h"
#include "AtlasVectorTypes.h"
#include "simbicon/Controller.h"

Controller *simbiconController;

extern "C" {

//////////////////////////////////////////////////
AtlasSimInterface* create_atlas_sim_interface()
{
  std::cerr << "\nWarning: Using Atlas Shim interface. "
    << "Atlas will be uncontrolled\n";

  std::vector<double> p;
  p.resize(Atlas::NUM_JOINTS);
  std::vector<double> v;
  v.resize(Atlas::NUM_JOINTS);
  simbiconController = new Controller(p, v);
  return new AtlasSimInterface();
}

//////////////////////////////////////////////////
void destroy_atlas_sim_interface()
{
  delete simbiconController;
}

} // end extern "C"

//////////////////////////////////////////////////
AtlasSimInterface::AtlasSimInterface()
{
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_major()
{
  return 2;
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_minor()
{
  return 10;
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_point()
{
  // The -1 means that this library is just a shim library and it doesn't
  // provide real functionality.
  return -1;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::process_control_input(
    const AtlasControlInput& control_input,
    const AtlasRobotState& robot_state,
    AtlasControlOutput& control_output)
{

  // Joint [0-5]: FreeJoint (6) Child body : pelvis
  // Joint [6]: back_bkz (1) Parent body: pelvis Child body : ltorso 
  // Joint [7]: l_leg_hpz (1) Parent body: pelvis Child body : l_uglut 
  // Joint [8]: r_leg_hpz (1) Parent body: pelvis Child body : r_uglut 
  // Joint [9]: back_bky (1) Parent body: ltorso Child body : mtorso 
  // Joint [10]: l_leg_hpx (1) Parent body: l_uglut Child body : l_lglut 
  // Joint [11]: r_leg_hpx (1) Parent body: r_uglut Child body : r_lglut 
  // Joint [12]: back_bkx (1) Parent body: mtorso Child body : utorso 
  // Joint [13]: l_leg_hpy (1) Parent body: l_lglut Child body : l_uleg 
  // Joint [14]: r_leg_hpy (1) Parent body: r_lglut Child body : r_uleg 
  // Joint [15]: l_arm_shy (1) Parent body: utorso Child body : l_clav 
  // Joint [16]: r_arm_shy (1) Parent body: utorso Child body : r_clav 
  // Joint [17]: l_leg_kny (1) Parent body: l_uleg Child body : l_lleg 
  // Joint [18]: r_leg_kny (1) Parent body: r_uleg Child body : r_lleg 
  // Joint [19]: l_arm_shx (1) Parent body: l_clav Child body : l_scap 
  // Joint [20]: r_arm_shx (1) Parent body: r_clav Child body : r_scap 
  // Joint [xx]: l_leg_aky (1) Parent body: l_lleg Child body : l_talus 
  // Joint [xx]: r_leg_aky (1) Parent body: r_lleg Child body : r_talus 
  // Joint [xx]: l_arm_ely (1) Parent body: l_scap Child body : l_uarm 
  // Joint [xx]: r_arm_ely (1) Parent body: r_scap Child body : r_uarm 
  // Joint [xx]: l_leg_akx (1) Parent body: l_talus Child body : l_foot 
  // Joint [xx]: r_leg_akx (1) Parent body: r_talus Child body : r_foot 
  // Joint [xx]: l_arm_elx (1) Parent body: l_uarm Child body : l_larm 
  // Joint [xx]: r_arm_elx (1) Parent body: r_uarm Child body : r_larm 
  // Joint [xx]: l_arm_wry (1) Parent body: l_larm

  // convert control_input to controller usable data

  // convert robot_state to controller usable data

  const double dt = 0.001;
  std::vector<double> torques = simbiconController->update(dt);

  // copy control torque to control_output
  for (int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    control_output.f_out[i] = torques[i];
    control_output.f_out[i] = 10;
  }

  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::reset_control()
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::set_desired_behavior(
    const std::string& behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_desired_behavior(
    std::string& desired_behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_current_behavior(
    std::string& current_behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_num_behaviors(int& num_behaviors)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_behavior_at_index(int index,
    std::string& behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_behavior_joint_weights(
    const std::string& behavior,
    float joint_control_weights[NUM_JOINTS])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_current_behavior_joint_weights(
    float joint_control_weights[NUM_JOINTS])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_estimated_position(
    AtlasPositionData& robot_pos_est,
    AtlasVec3f foot_pos_est[Atlas::NUM_FEET])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
std::string AtlasSimInterface::get_error_code_text(AtlasErrorCode ec)
{
  return std::string("NO ERRORS");
}
