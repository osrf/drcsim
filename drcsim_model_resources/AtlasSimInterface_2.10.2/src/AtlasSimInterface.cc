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

Controller simbiconController;

extern "C" {

//////////////////////////////////////////////////
AtlasSimInterface* create_atlas_sim_interface()
{
  std::cerr << "\nWarning: Using Atlas Shim interface. "
    << "Atlas will be uncontrolled\n";
  return new AtlasSimInterface();
}

//////////////////////////////////////////////////
void destroy_atlas_sim_interface()
{
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
  const double dt = 0.001;
  simbiconController.update(dt);
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
