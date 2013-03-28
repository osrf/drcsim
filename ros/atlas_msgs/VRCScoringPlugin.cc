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

#include <gazebo/common/Common.hh>
#include <gazebo/physics/Physics.hh>
#include "VRCScoringPlugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
VRCScoringPlugin::VRCScoringPlugin()
{
}

/////////////////////////////////////////////////
VRCScoringPlugin::~VRCScoringPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void VRCScoringPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _world;
  this->atlas = _world->GetModel("atlas");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VRCScoringPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VRCScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  common::Time curTime = this->world->GetSimTime();
  if (!this->atlas)
  {
    this->atlas = this->world->GetModel("atlas");
    return;
  }

  common::Time diffTime = _info.simTime - this->prevTime;

  gazebo::physics::LinkPtr link = this->atlas->GetLink("head");

  if (!link)
    std::cout << "Unable to find head\n";

  std::cout << _info.simTime.Double() << " "
    << link->GetWorldLinearVel().z / diffTime.Double() << std::endl;

  this->prevTime = _info.simTime;
}

GZ_REGISTER_WORLD_PLUGIN(VRCScoringPlugin)
