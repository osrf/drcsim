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

#include "drcsim_gazebo_ros_plugins/test_ros_plugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
TestROSPlugin::TestROSPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TestROSPlugin::~TestROSPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void TestROSPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Parse _sdf to get parameters for this plugin if you like

  // Get ponters to world and model
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&TestROSPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void TestROSPlugin::UpdateStates()
{
}

GZ_REGISTER_MODEL_PLUGIN(TestROSPlugin)
}
