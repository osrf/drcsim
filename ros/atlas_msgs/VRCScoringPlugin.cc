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
  this->deferredLoadThread.join();
  // Be sure to write the final score data before quitting
  this->WriteScore(this->world->GetSimTime(), "Shutting down", true);
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

  // Everybody needs Atlas.
  this->atlas = _world->GetModel("atlas");
  if (!this->atlas)
  {
    gzerr << "Failed to find atlas" << std::endl;
    return;
  }
  this->atlasHead = this->atlas->GetLink("head");
  if (!this->atlasHead)
  {
    gzerr << "Unable to find head for scoring falls" << std::endl;
    return;
  }

  // Arena-specific setup
  switch (this->worldType)
  {
    case QUAL_1:
      if (!this->FindGates())
        return;
      break;
    case QUAL_2:
      if (!this->FindQual2Stuff())
        return;
      break;
    case QUAL_3:
      if (!this->FindGates())
        return;
      break;
    case QUAL_4:
      if (!this->FindGates())
        return;
      break;
    case VRC_1:
      if (!this->FindVRC1Stuff())
        return;
      break;
    case VRC_2:
      if (!this->FindGates())
        return;
      break;
    case VRC_3:
      if (!this->FindVRC3Stuff())
        return;
      break;
    default:
      GZ_ASSERT(false, "Unknown worldType");
  }

  this->prevVelTime = common::Time(0,0);
  this->prevScoreTime = common::Time(0,0);
  this->prevFallTime = common::Time(0,0);

  this->completionScore = 0;
  this->falls = 0;

  if (_sdf->HasElement("fall_accel_threshold"))
    this->fallAccelThreshold = _sdf->GetValueDouble("fall_accel_threshold");
  else
    this->fallAccelThreshold = 1000.0;

  if (_sdf->HasElement("score_file"))
    this->scoreFilePath =
      boost::filesystem::path(_sdf->GetValueString("score_file"));
  else
  {
    // Get the user's home directory
    // \todo getenv is not portable, and there is no generic cross-platform
    // method. Must check OS and choose a method
    char *homePath = getenv("HOME");

    if (!homePath)
      this->scoreFilePath = boost::filesystem::path("/tmp/gazebo");
    else
      this->scoreFilePath = boost::filesystem::path(homePath);

    this->scoreFilePath /= ".gazebo";
    this->scoreFilePath /= "scores";
    this->scoreFilePath /= this->world->GetName() + ".score";
  }

  // Create the score directory if needed
  if (!boost::filesystem::exists(this->scoreFilePath.parent_path()))
    boost::filesystem::create_directories(this->scoreFilePath.parent_path());
  // Open the score file for writing
  this->scoreFileStream.open(this->scoreFilePath.string().c_str(),
                             std::fstream::out);
  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Failed to open score file :" << this->scoreFilePath <<
      std::endl;
    return;
  }
  gzlog << "Writing score data to " << this->scoreFilePath << std::endl;

  this->scoreFileStream << "# Score data for world " <<
    this->world->GetName() << std::endl;
  this->scoreFileStream << "# Started at: " <<
    std::fixed << std::setprecision(3) <<
    common::Time::GetWallTime().Double() << std::endl;
  this->scoreFileStream << "# Format: " << std::endl;
  this->scoreFileStream << "# wallTime(sec),simTime(sec),"
    "wallTimeElapsed(sec),simTimeElapsed(sec),completionScore(count),"
    "falls(count)" << std::endl;

  this->deferredLoadThread =
    boost::thread(boost::bind(&VRCScoringPlugin::DeferredLoad, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VRCScoringPlugin::OnUpdate, this, _1));
}

////////////////////////////////////////////////////////////////////////////////
void VRCScoringPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading VRC scoring plugin because ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // publish multi queue
  this->pmq.startServiceThread();

  this->pubScoreQueue = this->pmq.addPub<atlas_msgs::VRCScore>();
  this->pubScore = this->rosNode->advertise<atlas_msgs::VRCScore>(
    "vrc_score", 1, true);
}

/////////////////////////////////////////////////
void VRCScoringPlugin::WriteScore(const common::Time& _currTime,
  const std::string &_msg, bool _force)
{
  // Write at 1Hz
  if (!_force && (_currTime - this->prevScoreTime).Double() < 1.0)
    return;

  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Score file stream is no longer open:" << this->scoreFilePath <<
      std::endl;
    return;
  }

  common::Time currentWallTime = common::Time::GetWallTime();

  // If we've passed the first gate, compute elapsed time
  common::Time elapsedTimeSim;
  if (!math::equal(this->startTimeSim.Double(), 0.0))
    elapsedTimeSim = _currTime - this->startTimeSim;
  common::Time elapsedTimeWall;
  if (!math::equal(this->startTimeWall.Double(), 0.0))
    elapsedTimeWall = currentWallTime - this->startTimeWall;

  this->scoreFileStream << std::fixed << std::setprecision(3)
    << currentWallTime.Double() << ","
    << _currTime.Double() << ","
    << elapsedTimeWall.Double() << ","
    << elapsedTimeSim.Double() << ","
    << this->completionScore << ","
    << this->falls << ",\"" << _msg << "\"" << std::endl;

  // Also publish via ROS
  atlas_msgs::VRCScore rosScoreMsg;
  rosScoreMsg.wall_time = ros::Time(common::Time::GetWallTime().Double());
  rosScoreMsg.sim_time = ros::Time(_currTime.Double());
  rosScoreMsg.wall_time_elapsed = ros::Time(elapsedTimeWall.Double());
  rosScoreMsg.sim_time_elapsed = ros::Time(elapsedTimeSim.Double());
  rosScoreMsg.completion_score = this->completionScore;
  rosScoreMsg.falls = this->falls;
  rosScoreMsg.message = _msg;
  this->pubScoreQueue->push(rosScoreMsg, this->pubScore);

  this->prevScoreTime = _currTime;
}

/////////////////////////////////////////////////
int VRCScoringPlugin::IsPoseInGate(const math::Pose& _robotWorldPose,
                                   const math::Pose& _gateWorldPose,
                                   double _gateWidth)
{
  // Transform to gate frame
  math::Vector3 robotLocalPosition =
    _gateWorldPose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    _gateWorldPose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= _gateWidth / 2.0)
    return (robotLocalPosition.x >= 0.0) ? 1 : -1;
  else
    return 0;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckNextGate(std::string &_msg)
{
  if (this->nextGate != this->gates.end())
  {
    // Get the pose of the robot or the vehicle, depending on the type of the
    // gate.
    math::Pose pose;
    switch (this->nextGate->type)
    {
      case Gate::PEDESTRIAN:
        pose = this->atlas->GetWorldPose();
        break;
      case Gate::VEHICLE:
        pose = this->vehicle->GetWorldPose();
        break;
      default:
        GZ_ASSERT(false, "Unknown gate type");
    }
    // Figure whether we're positioned before (-1), after (1), or
    // neither (0), with respect to the gate.
    int gateSide = this->IsPoseInGate(pose,
                                      this->nextGate->pose,
                                      this->nextGate->width);
    // Did we go forward through the gate?
    if ((this->nextGateSide < 0) && (gateSide > 0))
    {
      // Log it
      std::stringstream ss;
      ss << "Successfully passed through gate " <<
        this->nextGate->number;
      gzlog << ss.str() << std::endl;
      _msg += ss.str();

      // If it's the first gate, note the time
      if (this->nextGate == this->gates.begin())
      {
        this->startTimeSim = this->world->GetSimTime();
        this->startTimeWall = common::Time::GetWallTime();
      }

      // Update state to look for the next gate
      ++this->nextGate;
      this->nextGateSide = 0;
      return true;
    }
    else
    {
      // Just checking: did we go backward through the gate?
      if ((this->nextGateSide > 0) && (gateSide < 0))
      {
        gzlog << "Went backward through gate " <<
          this->nextGate->number << std::endl;
      }
      // Remember which side we're on now (which might be 0)
      this->nextGateSide = gateSide;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckDrillInBin(std::string &_msg)
{
  // Only report the first time
  if (this->completionScore == 0)
  {
    math::Vector3 drillPosition = this->drill->GetWorldPose().pos;
    if ((drillPosition.x >= this->bin.min.x) &&
        (drillPosition.x <= this->bin.max.x) &&
        (drillPosition.y >= this->bin.min.y) &&
        (drillPosition.y <= this->bin.max.y) &&
        (drillPosition.z >= this->bin.min.z) &&
        (drillPosition.z <= this->bin.max.z))
    {
      gzlog << "Successfully placed drill in bin" << std::endl;
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckFall(const common::Time &_currTime,
  std::string &_msg)
{
  // Get head velocity
  math::Vector3 currVel = this->atlasHead->GetWorldLinearVel();

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
    std::stringstream ss;
    ss << "Damaging fall detected, acceleration of: " << accel <<
      " m/s^2";
    gzlog << ss.str() << std::endl;
    _msg += ss.str();
    this->prevFallTime = _currTime;
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseOffTable(std::string &_msg)
{
  // Check that the height of the hose couple is within a few cm
  // of the standpipe.
  math::Vector3 standpipePosition = this->standpipe->GetWorldPose().pos;
  math::Vector3 hoseCouplerPosition = this->hoseCoupler->GetWorldPose().pos;
  if (hoseCouplerPosition.z >= (standpipePosition.z - 0.05))
  {
    std::stringstream ss;
    ss << "Successfully picked hose up off table";
    gzlog << ss.str() << std::endl;
    _msg += ss.str();
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseAligned(std::string &_msg)
{
  // Check that the screw joint between the hose coupler and standpipe exists.
  // That's true only when they're aligned (handled in VRCPlugin.cpp).
  // We check indirectly by looking for a non-empty set of child links attached
  // to the standpipe.
  physics::Link_V childJointsLinks = this->standpipe->GetChildJointsLinks();
  if (!childJointsLinks.empty())
  {
    // If we were not previously aligned, latch the current pose for later
    // comparison to determine that rotation has succeeded.
    if (!this->isHoseAligned)
    {
      std::stringstream ss;
      ss << "Successfully aligned the hose with the standpipe";
      gzlog << ss.str() << std::endl;
      _msg += ss.str();
      this->hoseCouplerAlignedPose = this->hoseCoupler->GetWorldPose();
      this->isHoseAligned = true;
      this->isHoseConnected = false;
    }
    return true;
  }
  else
  {
    if (this->isHoseAligned)
    {
      gzlog << "Unaligned the hose from the standpipe" << std::endl;
      this->isHoseAligned = false;
      this->isHoseConnected = false;
    }
    return false;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseConnected(std::string &_msg)
{
  // Must be aligned (i.e., the screw joint must exist)
  if (!this->isHoseAligned)
    return false;

  // Check for a sufficient change in coupler position along its X axis,
  // which indicates that the hose coupler has threaded on to the standpipe
  // screw joint.

  // Transform current position into frame of initial aligned pose
  math::Vector3 couplerWorldPosition = this->hoseCoupler->GetWorldPose().pos;
  math::Vector3 couplerLocalPosition =
    this->hoseCouplerAlignedPose.rot.GetInverse().RotateVector(
      couplerWorldPosition - this->hoseCouplerAlignedPose.pos);
  double dist = couplerLocalPosition.x;
  // Maximum depth is 2cm; let's get most of the way there.
  if (dist <= -0.015)
  {
    if (!this->isHoseConnected)
    {
      std::stringstream ss;
      ss << "Successfully connected the hose to the standpipe";
      gzlog << ss.str() << std::endl;
      _msg += ss.str();
      this->isHoseConnected = true;
    }
    return true;
  }
  else
  {
    if (this->isHoseConnected)
    {
      gzlog << "Disconnected the hose to the standpipe" << std::endl;
      this->isHoseConnected = false;
    }
    return false;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckValveOpen(std::string &_msg)
{
  // Doesn't count unless the hose is connected.  This check doesn't
  // prevent out-of-order task execution; that should be done in
  // the VRCPlugin, which should not allow alignment when the valve is open.
  if (!this->isHoseConnected)
    return false;

  // The valve starts at 0 and can be turned CCW several rotations.
  // We check that it's been turned at least one rotation.
  if (this->valve->GetAngle(0) < math::Angle(-2.0*M_PI))
  {
    std::stringstream ss;
    ss << "Successfully opened valve";
    gzlog << ss.str() << std::endl;
    _msg += ss.str();
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////
void VRCScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  int prevScore = this->completionScore;
  int prevFalls = this->falls;
  std::string scoreMsg;
  bool forceLogScore = false;

  // Did we pass through a gate?
  if (this->IsGateBased() && this->CheckNextGate(scoreMsg))
    this->completionScore += 1;

  if (this->worldType == QUAL_2)
  {
    if (this->completionScore == 0)
    {
      // Did we put the drill in the bin?
      if (this->CheckDrillInBin(scoreMsg))
        this->completionScore += 1;
    }
  }
  else if (this->worldType == VRC_3)
  {
    // Check for passing the first gate; doesn't affect score, but we need to
    // latch sim and wall time of that event.
    if (this->CheckNextGate(scoreMsg))
    {
      // Force score output so that this event appears in the log
      forceLogScore = true;
    }

    // Check for alignment and connection every cycle, because the
    // competitor might align/connect and unalign/disconnect the hose
    // multiple times.
    bool hoseAligned = this->CheckHoseAligned(scoreMsg);
    bool hoseConnected = this->CheckHoseConnected(scoreMsg);

    if (this->completionScore == 0)
    {
      // Step 1: Did we get the hose up off the table?
      if (this->CheckHoseOffTable(scoreMsg))
        this->completionScore += 1;
    }
    else if (this->completionScore == 1)
    {
      // Step 2: Did we align the hose?
      if (hoseAligned)
        this->completionScore += 1;
    }
    else if (this->completionScore == 2)
    {
      // Step 3: Did we connect the hose?
      if (hoseConnected)
        this->completionScore += 1;
    }
    else if (this->completionScore == 3)
    {
      // Step 3: Did we turn the valve?
      if (this->CheckValveOpen(scoreMsg))
        this->completionScore += 1;
    }
  }

  // Did we fall?
  if (this->CheckFall(_info.simTime, scoreMsg))
    this->falls += 1;

  // Write score data, forcing a write if any score changed;
  // when not forced, it's throttled internally to
  // write at a fixed rate.
  if ((prevScore != this->completionScore) ||
      (prevFalls != this->falls))
    forceLogScore = true;
  this->WriteScore(_info.simTime, scoreMsg, forceLogScore);
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindQual2Stuff()
{
  this->drill = this->world->GetModel("drill");
  if (!this->drill)
  {
    gzerr << "Failed to find drill" << std::endl;
    return false;
  }
  physics::ModelPtr bin = this->world->GetModel("bin");
  if (!bin)
  {
    gzerr << "Failed to find bin" << std::endl;
    return false;
  }

  // Determine the bbox we need the drill to be within
  physics::LinkPtr binLink = bin->GetLink("link");
  if (!binLink)
  {
    gzerr << "Failed to find bin link" << std::endl;
    return false;
  }
  physics::CollisionPtr bottomCollision =
    binLink->GetCollision("bottom_collision");
  if (!bottomCollision)
  {
    gzerr << "Failed to find bin bottom collision" << std::endl;
    return false;
  }
  math::Box bottomBbox = bottomCollision->GetBoundingBox();
  physics::CollisionPtr side1Collision =
    binLink->GetCollision("side1_collision");
  if (!side1Collision)
  {
    gzerr << "Failed to find bin side1 collision" << std::endl;
    return false;
  }
  math::Box side1Bbox = side1Collision->GetBoundingBox();

  this->bin.min.x = bottomBbox.min.x;
  this->bin.min.y = bottomBbox.min.y;
  // Give a bit of tolerance for possible offset in origin of drill
  this->bin.min.z = bottomBbox.min.z - 0.15;
  this->bin.max.x = bottomBbox.max.x;
  this->bin.max.y = bottomBbox.max.y;
  this->bin.max.z = side1Bbox.max.z;

  return true;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindVRC1Stuff()
{
  this->vehicle = this->world->GetModel("drc_vehicle");
  if (!this->vehicle)
  {
    gzerr << "Failed to find vehicle" << std::endl;
    return false;
  }
  if (!this->FindGates())
    return false;
  return true;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindVRC3Stuff()
{
  physics::ModelPtr hose = this->world->GetModel("vrc_firehose_long");
  if (!hose)
  {
    gzerr << "Failed to find hose" << std::endl;
    return false;
  }
  this->hoseCoupler = hose->GetLink("coupling");
  if (!this->hoseCoupler)
  {
    gzerr << "Failed to find hose coupler" << std::endl;
    return false;
  }

  physics::ModelPtr standpipeModel = this->world->GetModel("standpipe");
  if (!standpipeModel)
  {
    gzerr << "Failed to find standpipe model" << std::endl;
    return false;
  }
  this->standpipe = standpipeModel->GetLink("standpipe");
  if (!this->standpipe)
  {
    gzerr << "Failed to find standpipe link" << std::endl;
    return false;
  }

  physics::ModelPtr valveModel = this->world->GetModel("valve");
  if (!valveModel)
  {
    gzerr << "Failed to find valve model" << std::endl;
    return false;
  }
  this->valve = valveModel->GetJoint("valve");
  if (!this->valve)
  {
    gzerr << "Failed to find valve joint" << std::endl;
    return false;
  }

  this->isHoseAligned = false;
  this->isHoseConnected = false;

  if (!this->FindGates())
    return false;

  return true;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindGates()
{
  // Walk through the world and accumulate the things that appear to be gates.
  physics::Model_V models = this->world->GetModels();
  for (physics::Model_V::const_iterator it = models.begin();
       it != models.end();
       ++it)
  {
    // Parse the name, assuming that gates are named '[vehicle]gate_<int>'
    physics::ModelPtr model = *it;
    std::string name = model->GetName();
    std::vector<std::string> parts;
    boost::split(parts, name, boost::is_any_of("_"));
    if (parts.size() == 2 &&
        ((parts[0] == "gate") || (parts[0] == "vehiclegate")))
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
      math::Box bbox = model->GetBoundingBox();
      math::Vector3 bboxSize = bbox.GetSize();
      double gateWidth = std::max(bboxSize.x, bboxSize.y);

      Gate::GateType gateType;
      if (parts[0] == "vehiclegate")
        gateType = Gate::VEHICLE;
      else
        gateType = Gate::PEDESTRIAN;

      // Store this gate
      Gate g(name, gateType, gateNum, model->GetWorldPose(), gateWidth);
      this->gates.push_back(g);
      gzlog << "Stored gate named " << g.name << " of type " << g.type
        << " with index " << g.number << " at pose " << g.pose
        << " and width " << g.width << std::endl;
    }
  }

  if (this->gates.empty())
  {
    gzerr << "Found no gates." << std::endl;
    this->nextGate = this->gates.end();
    return false;
  }

  // Sort in order of increasing gate number (in case we encountered them
  // out-of-order in the list of models).
  this->gates.sort();
  // Set the first gate we're looking for
  this->nextGate = this->gates.begin();
  this->nextGateSide = -1;

  return true;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::IsGateBased()
{
  if (this->worldType == QUAL_1 ||
      this->worldType == QUAL_3 ||
      this->worldType == QUAL_4 ||
      this->worldType == VRC_1 ||
      this->worldType == VRC_2)
    return true;
  else
    return false;
}

GZ_REGISTER_WORLD_PLUGIN(VRCScoringPlugin)
