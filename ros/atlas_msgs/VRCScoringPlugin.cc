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
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  // Create the score publisher. Only send out data at 10 Hz.
  this->scorePub = this->node->Advertise<gazebo::msgs::GzString>(
      "~/vrc/score", 100, 10);

  // Create the time publisher. Only send out data at 30 Hz.
  this->timePub = this->node->Advertise<gazebo::msgs::Time>(
      "~/vrc/time", 100, 30);

  // Get the world name.
  this->world = _world;
  this->atlas = _world->GetModel("atlas");

  this->score = 0.0;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VRCScoringPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VRCScoringPlugin::UpdateScore()
{
  this->score += 0.1;

  gazebo::math::Pose atlasPose = this->atlas->GetWorldPose();

  /// Check all the gates.
  for (std::list<Gate>::iterator iter = this->gates.begin();
       iter != this->gates.end(); ++iter)
  {
  }

  this->prevAtlasPose = altasPose;
}

/////////////////////////////////////////////////
void VRCScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  common::Time curTime = this->world->GetSimTime();
  if (!this->atlas)
  {
    this->atlas = this->world->GetModel("atlas");

    // Grab the current pose of Atlas if found.
    if (this->atlas)
      this->atlasPrevPose = this->atlas->GetWorldPose();
    else
      return;
  }

  common::Time diffTime = _info.simTime - this->prevTime;

  gazebo::physics::LinkPtr link = this->atlas->GetLink("head");

  if (!link)
    std::cout << "Unable to find head\n";

  std::cout << _info.simTime.Double() << " "
    << link->GetWorldLinearVel().z / diffTime.Double() << std::endl;

  this->prevTime = _info.simTime;

  // Update the current score.
  this->UpdateScore();

  // Publish data
  {
    gazebo::msgs::GzString scoreMsg;
    scoreMsg.set_data(boost::lexical_cast<std::string>(this->score));
    this->scorePub->Publish(scoreMsg);

    gazebo::msgs::Time timeMsg;
    if (this->startTime >= _info.simTime)
      gazebo::msgs::Set(timeMsg, this->startTime - _info.simTime);
    this->timePub->Publish(timeMsg);
  }
}

GZ_REGISTER_WORLD_PLUGIN(VRCScoringPlugin)
