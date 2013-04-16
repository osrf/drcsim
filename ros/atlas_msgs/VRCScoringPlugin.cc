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

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "VRCScoringPlugin.hh"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <vector>

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
  // Which type of world are we scoring?
  this->world = _world;
  gzlog << "VRCScoringPlugin: world name is \"" <<
    this->world->GetName() << "\"" << std::endl;
  if (this->world->GetName() == "qual_task_1")
    this->worldType = QUAL_1;
  else if (this->world->GetName() == "qual_task_2")
    this->worldType = QUAL_2;
  else if (this->world->GetName() == "qual_task_3")
    this->worldType = QUAL_3;
  else if (this->world->GetName() == "qual_task_4")
    this->worldType = QUAL_4;
  else if (this->world->GetName() == "vrc_task_1")
    this->worldType = VRC_1;
  else if (this->world->GetName() == "vrc_task_2")
    this->worldType = VRC_2;
  else if (this->world->GetName() == "vrc_task_3")
    this->worldType = VRC_3;
  else
  {
    gzerr << "VRCScoringPlugin: unknown world name \"" <<
      this->world->GetName() << "\"; not scoring.";
    return;
  }

  if (this->IsGateBased())
    this->FindGates();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Create the score publisher. Only send out data at 10 Hz.
  this->scorePub = this->node->Advertise<gazebo::msgs::GzString>(
      "~/vrc/score", 100, 10);

  // Create the time publisher. Only send out data at 30 Hz.
  this->timePub = this->node->Advertise<gazebo::msgs::Time>(
      "~/vrc/time", 100, 30);

  this->atlas = _world->GetModel("atlas");

  this->score = 0.0;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  //this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //boost::bind(&VRCScoringPlugin::OnUpdate, this, _1));
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

  this->prevAtlasPose = atlasPose;
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
      this->prevAtlasPose = this->atlas->GetWorldPose();
    else
      return;
  }

  common::Time diffTime = _info.simTime - this->prevTime;
  gazebo::physics::LinkPtr link = this->atlas->GetLink("head");

  if (!link)
    std::cout << "Unable to find head\n";

  gazebo::math::Vector3 currVel = link->GetWorldLinearVel();

  double velDiff = currVel.z - prevLinearVel.z;

  std::cerr << _info.simTime.Double() << " "
            << velDiff / diffTime.Double() << std::endl;

  this->prevTime = _info.simTime;
  this->prevLinearVel = currVel;

  // Update the current score.
  this->UpdateScore();

  // Publish data
  {
    gazebo::msgs::GzString scoreMsg;
    scoreMsg.set_data(boost::lexical_cast<std::string>(this->score));
    this->scorePub->Publish(scoreMsg);

    gazebo::msgs::Time timeMsg;
    if (this->startTime >= _info.simTime)
    {
      gazebo::common::Time diff = this->startTime - _info.simTime;
      timeMsg.set_sec(diff.sec);
      timeMsg.set_nsec(diff.nsec);
      this->timePub->Publish(timeMsg);
    }
  }
}

/////////////////////////////////////////////////
void VRCScoringPlugin::FindGates()
{
  // Walk through the world and accumulate the things that appear to be gates.
  gazebo::physics::Model_V models = this->world->GetModels();
  for (gazebo::physics::Model_V::const_iterator it = models.begin();
       it != models.end();
       ++it)
  {
    // Parse the name, assuming that gates are named 'gate_<int>'
    std::string name = (*it)->GetName();
    std::vector<std::string> parts;
    boost::split(parts, name, boost::is_any_of("_"));
    if (parts.size() == 2 && parts[0] == "gate")
    {
      unsigned int gate_num;
      try
      {
        gate_num = boost::lexical_cast<unsigned int>(parts[1]); 
        gzdbg << "Got gate num: " << gate_num << std::endl;
      }
      catch (const boost::bad_lexical_cast& e)
      {
        gzdbg << "Failed to parse " << name << std::endl;
      }
    }
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::IsGateBased()
{
  if (this->worldType == QUAL_1 ||
      this->worldType == QUAL_3 ||
      this->worldType == VRC_1 ||
      this->worldType == VRC_2)
    return true;
  else
    return false;
}

GZ_REGISTER_WORLD_PLUGIN(VRCScoringPlugin)
