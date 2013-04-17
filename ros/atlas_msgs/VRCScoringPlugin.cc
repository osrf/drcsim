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
#include <stdlib.h>

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

  this->atlas = _world->GetModel("atlas");

  this->prevVelTime = gazebo::common::Time(0,0);
  this->prevScoreTime = gazebo::common::Time(0,0);
  this->prevFallTime = gazebo::common::Time(0,0);

  this->completionScore = 0;
  this->falls = 0;

  if (_sdf->HasElement("fall_accel_threshold"))
    this->fallAccelThreshold = _sdf->GetValueDouble("fall_accel_threshold");
  else
    this->fallAccelThreshold = 100.0;

  if (_sdf->HasElement("score_file"))
    this->scoreFilePath =
      boost::filesystem::path(_sdf->GetValueString("score_file"));
  else
  {
    // Get the user's home directory
    // \todo getenv is not portable, and there is no generic cross-platform
    // method. Must check OS and choose a method
    char *homePath = getenv("HOME");
    GZ_ASSERT(homePath, "HOME environment variable is missing");

    if (!homePath)
      this->scoreFilePath = boost::filesystem::path("/tmp/gazebo");
    else
      this->scoreFilePath = boost::filesystem::path(homePath);
 
    this->scoreFilePath /= ".gazebo";
    this->scoreFilePath /= this->world->GetName() + ".score";
  }

  this->scoreFileStream.open(this->scoreFilePath.string().c_str(),
                             std::fstream::out);
  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Failed to open score file :" << this->scoreFilePath <<
      std::endl;
    return;
  }

  this->scoreFileStream << "# Score data for world " <<
    this->world->GetName() << std::endl;
  this->scoreFileStream << "# Started at: " <<
    gazebo::common::Time::GetWallTime().Double() << std::endl;
  this->scoreFileStream << "# Format: " << std::endl;
  this->scoreFileStream << "# wallTime(sec) simTime(sec) "
    "completionScore(count) falls(count)" << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VRCScoringPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VRCScoringPlugin::WriteIntermediateScore(
  const gazebo::common::Time& _currTime)
{
  // Write at 1Hz
  if ((_currTime - this->prevScoreTime).Double() < 1.0)
    return;

  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Score file stream is no longer open:" << this->scoreFilePath <<
      std::endl;
    return;
  }

  this->scoreFileStream << gazebo::common::Time::GetWallTime().Double() <<
    " " << _currTime.Double() << " " << this->completionScore << " " <<
    this->falls << std::endl;

  this->prevScoreTime = _currTime;
}

/////////////////////////////////////////////////
int VRCScoringPlugin::IsPoseInGate(const gazebo::math::Pose& _robotWorldPose,
                                   const gazebo::math::Pose& _gateWorldPose,
                                   double _gateWidth)
{
  // Transform to gate frame
  gazebo::math::Vector3 robotLocalPosition =
    _gateWorldPose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    _gateWorldPose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= _gateWidth / 2.0)
    return (robotLocalPosition.x >= 0.0) ? 1 : -1;
  else
    return 0;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckNextGate()
{
  if (this->nextGate != this->gates.end())
  {
    int gateSide = this->IsPoseInGate(this->atlas->GetWorldPose(),
                                     this->nextGate->pose,
                                     this->nextGate->width);
    // Did we go forward through the gate?
    if ((this->nextGateSide < 0) && (gateSide > 0))
    {
      gzlog << "Successfully passed through gate " <<
        this->nextGate->number << std::endl;
      // Update state to look for the next gate
      ++this->nextGate;
      this->nextGateSide = -1;
      return true;
    }
    else
    {
      // Just checking: did we go backward through the gate?
      if ((this->nextGateSide > 0) && (gateSide < 0))
      {
        gzwarn << "Went backward through gate " <<
          this->nextGate->number << std::endl;
      }
      // Remember which side we're on now (which might be 0)
      this->nextGateSide = gateSide;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckFall(const common::Time &_currTime)
{
  // Find the head and get its velocity
  gazebo::physics::LinkPtr link = this->atlas->GetLink("head");
  if (!link)
  {
    gzerr << "Unable to find head for scoring falls" << std::endl;
    return true;
  }
  gazebo::math::Vector3 currVel = link->GetWorldLinearVel();

  // Don't declare a fall if we had one recently.  This check also handles
  // initial conditions, which currently include dropping the robot onto
  // the ground at t=10
  if ((_currTime - this->prevFallTime).Double() < 15.0)
  {
    this->prevVelTime = _currTime;
    this->prevLinearVel = currVel;
    return false;
  }

  // Differentiate to get acceleration
  double dt = (_currTime - this->prevVelTime).Double();
  double accel = (currVel.z - prevLinearVel.z) / dt;
  this->prevVelTime = _currTime;
  this->prevLinearVel = currVel;
  if (fabs(accel) > this->fallAccelThreshold)
  {
    gzdbg << "Damaging fall detected, acceleration of: " << accel <<
      " m/s^2" << std::endl;
    this->prevFallTime = _currTime;
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////
void VRCScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Did we pass through a gate?
  if (this->IsGateBased() && this->CheckNextGate())
    this->completionScore += 1;

  // Did we fall?
  if (this->CheckFall(_info.simTime))
    this->falls += 1;

  // Write score data (it's throttled internally to write at a fixed rate)
  this->WriteIntermediateScore(_info.simTime);
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
    gazebo::physics::ModelPtr model = *it;
    std::string name = model->GetName();
    std::vector<std::string> parts;
    boost::split(parts, name, boost::is_any_of("_"));
    if (parts.size() == 2 && parts[0] == "gate")
    {
      // Parse out the number; skip if it fails
      unsigned int gateNum;
      try
      {
        gateNum = boost::lexical_cast<unsigned int>(parts[1]);
      }
      catch (const boost::bad_lexical_cast& e)
      {
        gzwarn << "Ignoring gate name that failed to parse: " << name <<
          std::endl;
        continue;
      }
      // Determine width of gate; it's the larger of the X and Y dimensions of
      // the bounding box of the gate.
      gazebo::math::Box bbox = model->GetBoundingBox();
      gazebo::math::Vector3 bboxSize = bbox.GetSize();
      double gateWidth = std::max(bboxSize.x, bboxSize.y);

      // Store this gate
      Gate g(name, gateNum, model->GetWorldPose(), gateWidth);
      this->gates.push_back(g);
      gzlog << "Stored gate named " << g.name << " with index " << g.number
        << " at pose " << g.pose << " and width " << g.width << std::endl;
    }
  }

  if (this->gates.empty())
  {
    gzwarn << "Found no gates." << std::endl;
    this->nextGate = this->gates.end();
    return;
  }

  // Sort in order of increasing gate number (in case we encountered them
  // out-of-order in the list of models).
  this->gates.sort();
  // Set the first gate we're looking for
  this->nextGate = this->gates.begin();
  this->nextGateSide = -1;
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
