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

#include <map>
#include <string>
#include <stdlib.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/CylinderShape.hh>
#include "drcsim_gazebo_ros_plugins/VRCPlugin.h"

namespace gazebo
{
GZ_REGISTER_WORLD_PLUGIN(VRCPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
VRCPlugin::VRCPlugin()
{
  /// initial anchor pose
  this->warpRobotWithCmdVel = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
VRCPlugin::~VRCPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world = _parent;
  this->sdf = _sdf;

  // By default, cheats are off.  Allow override via environment variable.
  char* cheatsEnabledString = getenv("VRC_CHEATS_ENABLED");
  if (cheatsEnabledString && (std::string(cheatsEnabledString) == "1"))
    this->cheatsEnabled = true;
  else
    this->cheatsEnabled = false;

  // ros callback queue for processing subscription
  // this->deferredLoadThread = boost::thread(
  //   boost::bind(&VRCPlugin::DeferredLoad, this));
  this->DeferredLoad();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading vrc plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // load VRC ROS API
  this->LoadVRCROSAPI();

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->robotCmdVel = geometry_msgs::Twist();

  // Load Vehicle
  this->drcVehicle.Load(this->world, this->sdf);

  // Load fire hose and standpipe
  this->drcFireHose.Load(this->world, this->sdf);

  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind(&VRCPlugin::ROSQueueThread, this));

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&VRCPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetRobotMode(_str->data);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotMode(const std::string &_str)
{
  if (_str == "no_gravity")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
  }
  else if (_str == "harnessed")
  {
    bool paused = this->world->IsPaused();
    this->world->SetPaused(true);

    // remove pin
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);

    // raise robot, find ground height, set it down and upright it, then pin it
    math::Pose atlasPose = this->atlas.pinLink->GetWorldPose();

    // where to raise robot to
    math::Pose atlasAway = atlasPose + math::Pose(0, 0, 50.0, 0, 0, 0);

    // move robot out of the way
    this->atlas.model->SetLinkWorldPose(atlasAway, this->atlas.pinLink);

    // where to start down casting ray to check for ground
    math::Pose rayStart = atlasPose - math::Pose(0, 0, -2.0, 0, 0, 0);

    physics::EntityPtr objectBelow =
      this->world->GetEntityBelowPoint(rayStart.pos);
    if (objectBelow)
    {
      math::Box groundBB = objectBelow->GetBoundingBox();
      double groundHeight = groundBB.max.z;

      // gzdbg << objectBelow->GetName() << "\n";
      // gzdbg << objectBelow->GetParentModel()->GetName() << "\n";
      // gzdbg << groundHeight << "\n";
      // gzdbg << groundBB.max.z << "\n";
      // gzdbg << groundBB.min.z << "\n";

      // slightly above ground and upright
      atlasPose.pos.z = groundHeight + 1.15;
      atlasPose.rot.SetFromEuler(0, 0, 0);
      this->atlas.model->SetLinkWorldPose(atlasPose, this->atlas.pinLink);

      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
      this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

      // turning off effect of gravity
      physics::Link_V links = this->atlas.model->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        links[i]->SetGravityMode(false);
      }
    }
    else
    {
      gzwarn << "No entity below robot, or GetEntityBelowPoint "
             << "returned NULL pointer.\n";
      // put atlas back
      this->atlas.model->SetLinkWorldPose(atlasPose, this->atlas.pinLink);
    }
    this->world->SetPaused(paused);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    if (!this->atlas.pinJoint)
      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
  }
  else if (_str == "pinned_with_gravity")
  {
    // pinning robot, and turning off effect of gravity
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    if (!this->atlas.pinJoint)
      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
  }
  else if (_str == "nominal")
  {
    // nominal
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
  }
  else if (_str == "bdi_stand")
  {
    // Robot is PID controlled in BDI stand Pose and PINNED.

    // pin robot
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    this->atlas.pinJoint = this->AddJoint(this->world,
                                      this->atlas.model,
                                      physics::LinkPtr(),
                                      this->atlas.pinLink,
                                      "revolute",
                                      math::Vector3(0, 0, 0),
                                      math::Vector3(0, 0, 1),
                                      0.0, 0.0);
    // turning off effect of gravity
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }

    // turn physics off while manipulating things
    bool physics = this->world->GetEnablePhysicsEngine();
    bool paused = this->world->IsPaused();
    this->world->SetPaused(true);
    this->world->EnablePhysicsEngine(false);

    // set robot configuration
    this->atlasCommandController.SetPIDStand(this->atlas.model);
    /// FIXME: uncomment sleep below and AtlasSimInterface fails to STAND, why?
    // gazebo::common::Time::Sleep(gazebo::common::Time(1.0));
    ROS_INFO("set robot configuration done");

    this->world->EnablePhysicsEngine(physics);
    this->world->SetPaused(paused);
    // this->atlasCommandController.SetBDIFREEZE();
  }
  else
  {
    ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobotWithCmdVel = false;
  }
  else
  {
    this->robotCmdVel = *_cmd;
    this->warpRobotWithCmdVel = true;
    this->lastUpdateTime = this->world->GetSimTime().Double();
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose)
{
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                     _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);

  // turn physics off during SetWorldPose
  {
    bool physics = this->world->GetEnablePhysicsEngine();
    bool paused = this->world->IsPaused();
    this->world->SetPaused(true);
    this->world->EnablePhysicsEngine(false);

    this->atlas.model->SetWorldPose(pose);

    this->world->EnablePhysicsEngine(physics);
    this->world->SetPaused(paused);
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &_cmd)
{
  math::Quaternion q(_cmd->orientation.w, _cmd->orientation.x,
                     _cmd->orientation.y, _cmd->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_cmd->position.x,
                                _cmd->position.y,
                                _cmd->position.z), q);
  /// \todo: get these from incoming message
  std::string gripperName = "r_hand";
  math::Pose relPose(math::Vector3(0, -0.3, -0.1),
               math::Quaternion(0, 0, 0));

  if (this->drcFireHose.fireHoseModel && this->drcFireHose.couplingLink)
  {
    physics::LinkPtr gripper = this->atlas.model->GetLink(gripperName);
    if (gripper)
    {
      // teleports the object being attached together
      pose = pose + relPose + gripper->GetWorldPose();
      this->drcFireHose.fireHoseModel->SetLinkWorldPose(pose,
        this->drcFireHose.couplingLink);

      if (!this->grabJoint)
        this->grabJoint = this->AddJoint(this->world, this->atlas.model,
                                         gripper,
                                         this->drcFireHose.couplingLink,
                                         "revolute",
                                         math::Vector3(0, 0, 0),
                                         math::Vector3(0, 0, 1),
                                         0.0, 0.0);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  this->RemoveJoint(this->grabJoint);
}

////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
physics::JointPtr VRCPlugin::AddJoint(physics::WorldPtr _world,
                                      physics::ModelPtr _model,
                                      physics::LinkPtr _link1,
                                      physics::LinkPtr _link2,
                                      std::string _type,
                                      math::Vector3 _anchor,
                                      math::Vector3 _axis,
                                      double _upper, double _lower,
                                      bool _disableCollision)
{
  physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
    _type, _model);
  joint->Attach(_link1, _link2);
  // load adds the joint to a vector of shared pointers kept
  // in parent and child links, preventing joint from being destroyed.
  joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
  // joint->SetAnchor(0, _anchor);
  joint->SetAxis(0, _axis);
  joint->SetHighStop(0, _upper);
  joint->SetLowStop(0, _lower);

  if (_link1)
    joint->SetName(_link1->GetName() + std::string("_") +
                              _link2->GetName() + std::string("_joint"));
  else
    joint->SetName(std::string("world_") +
                              _link2->GetName() + std::string("_joint"));
  joint->Init();


  // disable collision between the link pair
  if (_disableCollision)
  {
    if (_link1)
      _link1->SetCollideMode("fixed");
    if (_link2)
      _link2->SetCollideMode("fixed");
  }


  return joint;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_pose)
{
  // Check if drcVehicle.model is loaded
  if (!this->drcVehicle.model)
  {
    ROS_ERROR("drc_vehicle model not found, cannot enter car.");
    return;
  }
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                    _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);
  if (this->atlas.pinJoint)
    this->RemoveJoint(this->atlas.pinJoint);

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  // hardcoded offset of the robot when it's seated in the vehicle driver seat.
  this->atlas.vehicleRelPose = math::Pose(math::Vector3(-0.06, 0.3, 2.02),
                                              math::Quaternion());

  // turn physics off while manipulating things
  bool physics = this->world->GetEnablePhysicsEngine();
  bool paused = this->world->IsPaused();
  this->world->SetPaused(true);
  this->world->EnablePhysicsEngine(false);

  // set robot configuration
  this->atlasCommandController.SetSeatingConfiguration(this->atlas.model);
  ros::spinOnce();
  // give some time for controllers to settle
  // \todo: use joint state subscriber to check if goal is obtained
  gazebo::common::Time::MSleep(1000);
  ROS_INFO("set robot configuration done");

  this->world->EnablePhysicsEngine(physics);
  this->world->SetPaused(paused);

  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);

  // this->atlas.vehicleRelPose = math::Pose(math::Vector3(0.52, 0.5, 1.27),
  this->atlas.vehicleRelPose = math::Pose(-0.06, 0.3, 1.26, 0, 0, 0);

  this->RemoveJoint(this->vehicleRobotJoint);

  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotExitCar(const geometry_msgs::Pose::ConstPtr &_pose)
{
  // Check if drcVehicle.model is loaded
  if (!this->drcVehicle.model)
  {
    ROS_ERROR("drc_vehicle model not found, cannot exit car.");
    return;
  }
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                    _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);

  if (this->atlas.pinJoint)
    this->RemoveJoint(this->atlas.pinJoint);

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  // hardcoded offset of the robot when it's standing next to the vehicle.
  this->atlas.vehicleRelPose = math::Pose(0.52, 1.7, 1.20, 0, 0, 0);

  // turn physics off while manipulating things
  bool physics = this->world->GetEnablePhysicsEngine();
  bool paused = this->world->IsPaused();
  this->world->SetPaused(true);
  this->world->EnablePhysicsEngine(false);
  // set robot configuration
  this->atlasCommandController.SetStandingConfiguration(this->atlas.model);
  ros::spinOnce();
  // give some time for controllers to settle
  // \todo: use joint state subscriber to check if goal is obtained
  gazebo::common::Time::MSleep(1000);
  ROS_INFO("set configuration done");

  this->world->EnablePhysicsEngine(physics);
  this->world->SetPaused(paused);

  // move model to new pose
  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
  gazebo::common::Time::MSleep(5000);

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);
}


////////////////////////////////////////////////////////////////////////////////
// remove a joint
void VRCPlugin::RemoveJoint(physics::JointPtr &_joint)
{
  bool paused = this->world->IsPaused();
  this->world->SetPaused(true);
  if (_joint)
  {
    // reenable collision between the link pair
    physics::LinkPtr parent = _joint->GetParent();
    physics::LinkPtr child = _joint->GetChild();
    if (parent)
      parent->SetCollideMode("all");
    if (child)
      child->SetCollideMode("all");

    _joint->Detach();
    _joint.reset();
  }
  this->world->SetPaused(paused);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose,
                         const std::map<std::string, double> &/*_jp*/)
{
  this->Teleport(_pinLink, _pinJoint, _pose);
  /// \todo: use _jp to set robot configuration
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose)
{
  // pause, break joint, update pose, create new joint, unpause
  bool p = this->world->IsPaused();
  bool e = this->world->GetEnablePhysicsEngine();
  this->world->EnablePhysicsEngine(false);
  this->world->SetPaused(true);
  if (_pinJoint)
    this->RemoveJoint(_pinJoint);
  _pinLink->GetModel()->SetLinkWorldPose(_pose, _pinLink);
  if (!_pinJoint)
    _pinJoint = this->AddJoint(this->world,
                               _pinLink->GetModel(),
                               physics::LinkPtr(),
                               this->atlas.pinLink,
                               "revolute",
                               math::Vector3(0, 0, 0),
                               math::Vector3(0, 0, 1),
                               0.0, 0.0);
  this->world->SetPaused(p);
  this->world->EnablePhysicsEngine(e);
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void VRCPlugin::UpdateStates()
{
  double curTime = this->world->GetSimTime().Double();
  // if user chooses bdi_stand mode, robot will be initialized
  // with PID stand in BDI stand pose pinned.
  // After startupStandPrepDuration - 1 seconds, pin released.
  // After startupStandPrepDuration seconds, start Stand mode
  if (this->atlas.startupSequence == Robot::NONE)
  {
    // Load and Spawn Robot
    this->atlas.InsertModel(this->world, this->sdf);
    this->atlas.startupSequence = Robot::SPAWN_QUEUED;
  }
  else if (this->atlas.startupSequence == Robot::SPAWN_QUEUED)
  {
    if (this->atlas.CheckGetModel(this->world))
    {
      this->atlas.startupSequence = Robot::SPAWN_SUCCESS;
      this->atlasCommandController.InitModel(this->atlas.model);
    }
  }
  else if (this->atlas.startupSequence == Robot::SPAWN_SUCCESS)
  {
    // robot could have 2 distinct startup modes in sim:  bdi_stand | pinned
    // bdi_stand:
    //   Sets robot startup configuration to that of bdi stand behavior.
    //   PID robot at stand configuration for a few seconds.
    //   Turn on BDI controller, dynamically balance the robot.
    // pinned:
    //   Robot PID's to zero joint angles, and pinned to the world.
    //   If StartupHarnessDuration > 0 unpin the robot after duration.

    if (atlas.startupMode == "bdi_stand")
    {
      switch (this->atlas.bdiStandSequence)
      {
        case Robot::BS_NONE:
        {
          // ROS_INFO("BS_NONE");
          this->SetRobotMode("bdi_stand");
          // start the rest of the sequence
          this->atlas.bdiStandSequence = Robot::BS_PID_PINNED;
          this->atlas.startupBDIStandStartTime = this->world->GetSimTime();
          break;
        }
        case Robot::BS_PID_PINNED:
        {
          // ROS_INFO("BS_PID_PINNED");
          if ((curTime - this->atlas.startupBDIStandStartTime.Double()) >
            (atlas.startupStandPrepDuration - 8.0))
          {
            ROS_INFO("going into stand prep");
            this->atlasCommandController.SetBDIStandPrep();
            this->atlas.bdiStandSequence = Robot::BS_STAND_PREP_PINNED;
          }
          break;
        }
        case Robot::BS_STAND_PREP_PINNED:
        {
          // ROS_INFO("BS_STAND_PREP_PINNED");
          if ((curTime - this->atlas.startupBDIStandStartTime.Double()) >
            (atlas.startupStandPrepDuration - 0.3))
          {
            ROS_INFO("going into Nominal");
            this->SetRobotMode("nominal");
            this->atlas.bdiStandSequence = Robot::BS_STAND_PREP;
          }
          break;
        }
        case Robot::BS_STAND_PREP:
        {
          // ROS_INFO("BS_STAND_PREP");
          if ((curTime - this->atlas.startupBDIStandStartTime.Double()) >
              atlas.startupStandPrepDuration)
          {
            ROS_INFO("going into Dynamic Stand Behavior");
            this->atlasCommandController.SetBDIStand();
            this->atlas.bdiStandSequence = Robot::BS_INITIALIZED;
            this->atlas.startupSequence = Robot::INITIALIZED;
          }
          break;
        }
      }
    }
    else // if (atlas.startupMode == "pinned")
    {
      switch (this->atlas.pinnedSequence)
      {
        case Robot::PS_NONE:
        {
          ROS_DEBUG("Start robot with gravity turned off and harnessed.");
          this->SetRobotMode("pinned");
          if (math::equal(this->atlas.startupHarnessDuration, 0.0))
          {
            ROS_DEBUG("Atlas will stay pinned.");
            this->atlas.pinnedSequence = Robot::PS_INITIALIZED;
          }
          else
          {
            ROS_DEBUG("Resume to nominal mode after %f seconds.",
              this->atlas.startupHarnessDuration);
            this->atlas.pinnedSequence = Robot::PS_PINNED;
          }
          break;
        }
        case Robot::PS_PINNED:
        {
          // remove harness
          if (!math::equal(atlas.startupHarnessDuration, 0.0) &&
              curTime > atlas.startupHarnessDuration)
          {
            this->SetRobotMode("nominal");
            this->atlas.pinnedSequence = Robot::PS_INITIALIZED;
            this->atlas.startupSequence = Robot::INITIALIZED;
          }
          break;
        }
      }
    }
  }
  else if (this->atlas.startupSequence == Robot::INITIALIZED)
  {
    // done, do nothing
  }
  else
  {
    // should not be here
  }

  if (curTime > this->lastUpdateTime)
  {
    this->CheckThreadStart();

    double dt = curTime - this->lastUpdateTime;

    if (this->warpRobotWithCmdVel)
    {
      this->lastUpdateTime = curTime;
      math::Pose cur_pose = this->atlas.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->atlas.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      this->Teleport(this->atlas.pinLink,
                     this->atlas.pinJoint,
                     new_pose);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::ROSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::FireHose::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;

  sdf::ElementPtr sdf = _sdf->GetElement("drc_fire_hose");
  // Get special coupling links (on the firehose side)
  std::string fireHoseModelName = sdf->Get<std::string>("fire_hose_model");
  this->fireHoseModel = _world->GetModel(fireHoseModelName);
  if (!this->fireHoseModel)
  {
    ROS_DEBUG("fire_hose_model [%s] not found", fireHoseModelName.c_str());
    return;
  }
  this->initialFireHosePose = this->fireHoseModel->GetWorldPose();

  // Get coupling link
  std::string couplingLinkName = sdf->Get<std::string>("coupling_link");
  this->couplingLink = this->fireHoseModel->GetLink(couplingLinkName);
  if (!this->couplingLink)
  {
    ROS_ERROR("coupling link [%s] not found", couplingLinkName.c_str());
    return;
  }

  // Get joints
  this->fireHoseJoints = this->fireHoseModel->GetJoints();

  // Get links
  this->fireHoseLinks = this->fireHoseModel->GetLinks();

  // Get special links from the standpipe
  std::string standpipeModelName = sdf->Get<std::string>("standpipe_model");
  this->standpipeModel = _world->GetModel(standpipeModelName);
  if (!this->standpipeModel)
  {
    ROS_ERROR("standpipe model [%s] not found", standpipeModelName.c_str());
    return;
  }

  // Get spout link
  std::string spoutLinkName = sdf->Get<std::string>("spout_link");
  this->spoutLink = this->standpipeModel->GetLink(spoutLinkName);
  if (!this->spoutLink)
  {
    ROS_ERROR("spout link [%s] not found", spoutLinkName.c_str());
    return;
  }

  // Get the valve model and its joint
  std::string valveModelName;
  if (sdf->HasElement("valve_model"))
    valveModelName = sdf->Get<std::string>("valve_model");
  else
    valveModelName = "valve";
  this->valveModel = _world->GetModel(valveModelName);
  if (!this->valveModel)
  {
    ROS_ERROR("valve model [%s] not found", valveModelName.c_str());
    return;
  }
  std::string valveJointName;
  if (sdf->HasElement("valve_joint"))
    valveJointName = sdf->Get<std::string>("valve_joint");
  else
    valveJointName = "valve";
  this->valveJoint = this->valveModel->GetJoint(valveJointName);
  if (!this->valveJoint)
  {
    ROS_ERROR("valve joint [%s] not found", valveJointName.c_str());
    return;
  }

  this->threadPitch = sdf->Get<double>("thread_pitch");

  this->couplingRelativePose =
    sdf->Get<gazebo::math::Pose>("coupling_relative_pose");

  // Set initial configuration
  this->SetInitialConfiguration();

  this->isInitialized = true;
}

void VRCPlugin::FireHose::SetInitialConfiguration()
{
  // this does not work yet, because SetAngle only works for Hinge and Slider
  // joints, and fire hose is made of universal and ball joints.
  for (unsigned int i = 0; i < this->fireHoseJoints.size(); ++i)
  {
    // gzerr << "joint [" << this->fireHoseJoints[i]->GetName() << "]\n";
    this->fireHoseJoints[i]->SetAngle(0u, 0.0);
  }
}

void VRCPlugin::CheckThreadStart()
{
  if (!this->drcFireHose.isInitialized)
    return;

  // gzerr << "coupling [" << this->couplingLink->GetWorldPose() << "]\n";
  // gzerr << "spout [" << this->spoutLink->GetWorldPose() << "]\n"
  math::Pose connectPose(this->drcFireHose.couplingRelativePose);

  // surface of the coupling cylinder is -0.135m from link origin
  physics::CollisionPtr col =
    this->drcFireHose.couplingLink->GetCollision("attachment_col");
  double collisionSurfaceZOffset =
    col->GetRelativePose().pos.x -
    boost::dynamic_pointer_cast<physics::CylinderShape>(
    col->GetShape())->GetLength()/2;

  math::Pose relativePose =
    (math::Pose(collisionSurfaceZOffset, 0, 0, 0, 0, 0) +
     this->drcFireHose.couplingLink->GetWorldPose()) -
    this->drcFireHose.spoutLink->GetWorldPose();

  double posErrInsert = relativePose.pos.z - connectPose.pos.z +
    collisionSurfaceZOffset;
  double posErrCenter = fabs(relativePose.pos.x - connectPose.pos.x) +
                        fabs(relativePose.pos.y - connectPose.pos.y);
  double rotErr = (relativePose.rot.GetXAxis() -
                   connectPose.rot.GetXAxis()).GetLength();
  double valveAng = this->drcFireHose.valveJoint->GetAngle(0).Radian();

  // gzdbg << " connectPose [" << connectPose
  //       << "] [" << connectPose.rot.GetXAxis()
  //       << "] [" << connectPose.rot.GetYAxis()
  //       << "] [" << connectPose.rot.GetZAxis() << "]\n";
  // gzdbg << " relativePose [" << relativePose
  //       << "] [" << relativePose.rot.GetXAxis()  // bingo
  //       << "] [" << relativePose.rot.GetYAxis()
  //       << "] [" << relativePose.rot.GetZAxis() << "]\n";
  // math::Pose connectOffset = relativePose - connectPose;
  // gzdbg << "connect offset [" << connectOffset << "]\n";
  // gzerr << "insert [" << posErrInsert
  //       << "] center [" << posErrCenter
  //       << "] rpy [" << rotErr
  //       << "] valve [" << valveAng
  //       << "]\n";

  if (!this->drcFireHose.screwJoint)
  {
    // Check that the hose coupler is positioned within tolerance
    // and that the valve is not opened, because the water rushing out
    // would prevent you from attaching a hose.  This check also
    // prevents out-of-order execution that would confuse scoring in
    // VRCScoringPlugin.
    if (posErrInsert > 0.0 && posErrCenter < 0.003 &&
        rotErr < 0.05 && valveAng > -0.1)
    {
      this->drcFireHose.screwJoint =
        this->AddJoint(this->world, this->drcFireHose.fireHoseModel,
                       this->drcFireHose.spoutLink,
                       this->drcFireHose.couplingLink,
                       "screw",
                       math::Vector3(0, 0, 0),
                       math::Vector3(0, -1, 0),
                       20, -0.5, false);

      this->drcFireHose.screwJoint->SetAttribute("thread_pitch", 0,
        this->drcFireHose.threadPitch);

      // name of the joint
      // gzerr << this->drcFireHose.screwJoint->GetScopedName() << "\n";
    }
  }
  else
  {
    // check joint position to disconnect
    double position = this->drcFireHose.screwJoint->GetAngle(0).Radian();
    // gzdbg << "unscrew if [" <<  position << "] < -0.003\n";
    if (position < -0.0003)
      this->RemoveJoint(this->drcFireHose.screwJoint);
  }
}


////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Vehicle::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;
  // load parameters
  if (_sdf->HasElement("drc_vehicle") &&
      _sdf->GetElement("drc_vehicle")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("drc_vehicle")
                        ->Get<std::string>("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <drc_vehicle><model_name> blocks. using default.");
    this->model = _world->GetModel("drc_vehicle");
  }

  if (!this->model)
  {
    ROS_DEBUG("drc vehicle not found.");
    return;
  }

  if (_sdf->HasElement("drc_vehicle") &&
      _sdf->GetElement("drc_vehicle")->HasElement("seat_link"))
  {
    this->seatLink = this->model->GetLink(_sdf->GetElement("drc_vehicle")
                        ->Get<std::string>("seat_link"));
  }
  else
  {
    ROS_INFO("Can't find <drc_vehicle><seat_link> blocks, using default.");
    this->seatLink = this->model->GetLink("chassis");
  }

  if (!this->seatLink)
  {
    ROS_ERROR("drc vehicle seat link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->seatLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::Robot::Robot()
{
  this->startupSequence = Robot::NONE;
  this->bdiStandSequence = Robot::BS_NONE;
  this->pinnedSequence = Robot::PS_NONE;
}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::Robot::~Robot()
{
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Robot::InsertModel(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  // bunch of hardcoded presets
  this->startupHarnessDuration = 10;
  this->startupStandPrepDuration = 10;

  // changed by ros param
  this->spawnPose = math::Pose(0, 0, 0, 0, 0, 0);

  // default names, can be changed in SDF
  this->modelName = "atlas";
  this->pinLinkName = "utorso";
  std::string spawnPoseName = "robot_initial_pose";
  std::string robotDescriptionName = "robot_description";

  // load parameters
  if (_sdf->HasElement("atlas"))
  {
    sdf::ElementPtr atlasSDF = _sdf->GetElement("atlas");
    if (atlasSDF->HasElement("model_name"))
      this->modelName = atlasSDF->Get<std::string>("model_name");
    else
      ROS_INFO("Can't find <atlas><model_name> blocks. defaults to [atlas].");

    if (atlasSDF->HasElement("pin_link"))
      this->pinLinkName = atlasSDF->Get<std::string>("pin_link");
    else
      ROS_INFO("Can't find <atlas><pin_link> blocks, defaults to [utorso].");

    if (atlasSDF->HasElement("robot_description"))
      robotDescriptionName = atlasSDF->Get<std::string>("robot_description");
    else
      ROS_INFO("Can't find <atlas><robot_description> blocks, "
               "defaults to [robot_description].");

    // ros param name containing robot's initial pose
    if (atlasSDF->HasElement("robot_initial_pose"))
      spawnPoseName = atlasSDF->Get<std::string>("robot_initial_pose");
    else
      ROS_INFO("Can't find <atlas><pose> blocks, defaults to zero transform.");
  }
  else
    ROS_INFO("Can't find <atlas> blocks. using default: "
             "looking for model name [atlas], param [robot_description]"
             "link [utorso], param [robot_initial_pose/[x|y|z|roll|pitch|yaw]");

  // check if model exists already
  this->model = _world->GetModel(this->modelName);

  if (this->model)
  {
    ROS_INFO("atlas model found (included in world file).");
    this->startupSequence = Robot::SPAWN_SUCCESS;
  }
  else
  {
    ROS_INFO("atlas model not in world file, spawning from ros param [%s].",
      robotDescriptionName.c_str());

    // try spawn model from "robot_description" on ros parameter server
    ros::NodeHandle rh("");

    double x, y, z, roll, pitch, yaw;
    if (rh.getParam(spawnPoseName + "/x", x) &&
        rh.getParam(spawnPoseName + "/y", y) &&
        rh.getParam(spawnPoseName + "/z", z) &&
        rh.getParam(spawnPoseName + "/roll", roll) &&
        rh.getParam(spawnPoseName + "/pitch", pitch) &&
        rh.getParam(spawnPoseName + "/yaw", yaw))
    {
      this->spawnPose.pos = math::Vector3(x, y, z);
      this->spawnPose.rot = math::Vector3(roll, pitch, yaw);
    }
    else
      ROS_ERROR("robot initial spawn pose not found");

    std::string robotStr;
    if (rh.getParam(robotDescriptionName, robotStr))
    {
      // put model into gazebo factory queue (non-blocking)
      _world->InsertModelString(robotStr);
      this->startupSequence = Robot::SPAWN_QUEUED;
      ROS_INFO("atlas model pushed into gazebo spawn queue.");
    }
    else
    {
      ROS_ERROR("failed to spawn model from rosparam: [%s].",
        robotDescriptionName.c_str());
      return;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
bool VRCPlugin::Robot::CheckGetModel(physics::WorldPtr _world)
{
  this->model = _world->GetModel(this->modelName);
  if (this->model)
  {
    this->pinLink = this->model->GetLink(this->pinLinkName);

    if (!this->pinLink)
    {
      ROS_ERROR("atlas robot pin link not found, VRCPlugin will not work.");
      return false;
    }

    // initial pose specified by user in ros param under robot_initial_pose
    gzdbg << "spawnPose [" << this->spawnPose << "]\n";
    this->model->SetInitialRelativePose(this->spawnPose);
    this->model->SetWorldPose(this->spawnPose);

    // Note: hardcoded link by name: @todo: make this a pugin param
    this->initialPose = this->pinLink->GetWorldPose();

    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadVRCROSAPI()
{
  if (this->cheatsEnabled)
  {
    // ros subscription
    std::string robot_enter_car_topic_name = "drc_world/robot_enter_car";
    ros::SubscribeOptions robot_enter_car_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_enter_car_topic_name, 100,
      boost::bind(&VRCPlugin::RobotEnterCar, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotEnterCar = this->rosNode->subscribe(robot_enter_car_so);

    std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
    ros::SubscribeOptions robot_exit_car_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_exit_car_topic_name, 100,
      boost::bind(&VRCPlugin::RobotExitCar, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotExitCar = this->rosNode->subscribe(robot_exit_car_so);

    std::string robot_grab_topic_name = "drc_world/robot_grab_link";
    ros::SubscribeOptions robot_grab_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_grab_topic_name, 100,
      boost::bind(&VRCPlugin::RobotGrabFireHose, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotGrab = this->rosNode->subscribe(robot_grab_so);

    std::string robot_release_topic_name = "drc_world/robot_release_link";
    ros::SubscribeOptions robot_release_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_release_topic_name, 100,
      boost::bind(&VRCPlugin::RobotReleaseLink, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotRelease = this->rosNode->subscribe(robot_release_so);
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadRobotROSAPI()
{
  if (!this->rosNode->getParam("atlas/time_to_unpin",
    atlas.startupHarnessDuration))
  {
    ROS_DEBUG("atlas/time_to_unpin not specified, default harness duration to"
             " %f seconds", atlas.startupHarnessDuration);
  }

  if (!this->rosNode->getParam("atlas/startup_mode", atlas.startupMode))
  {
    ROS_INFO("atlas/startup_mode not specified, default bdi_stand that "
             " takes %f seconds to finish.", atlas.startupStandPrepDuration);
  }
  else if (atlas.startupMode == "bdi_stand")
  {
    ROS_INFO("Starting robot with BDI standing");
  }
  else if (atlas.startupMode == "pinned")
  {
    ROS_INFO("Starting robot pinned");
  }
  else
  {
    ROS_ERROR("Unsupported /atlas/startup_mode [%s]",
      atlas.startupMode.c_str());
  }

  if (this->cheatsEnabled)
  {
    // ros subscription
    std::string trajectory_topic_name = "atlas/cmd_vel";
    ros::SubscribeOptions trajectory_so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(
      trajectory_topic_name, 100,
      boost::bind(&VRCPlugin::SetRobotCmdVel, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subTrajectory = this->rosNode->subscribe(trajectory_so);

    std::string pose_topic_name = "atlas/set_pose";
    ros::SubscribeOptions pose_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      pose_topic_name, 100,
      boost::bind(&VRCPlugin::SetRobotPose, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subPose = this->rosNode->subscribe(pose_so);

    std::string configuration_topic_name = "atlas/configuration";
    ros::SubscribeOptions configuration_so =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
      configuration_topic_name, 100,
      boost::bind(&VRCPlugin::SetRobotConfiguration, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subConfiguration =
      this->rosNode->subscribe(configuration_so);

    std::string mode_topic_name = "atlas/mode";
    ros::SubscribeOptions mode_so =
      ros::SubscribeOptions::create<std_msgs::String>(
      mode_topic_name, 100,
      boost::bind(&VRCPlugin::SetRobotModeTopic, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subMode = this->rosNode->subscribe(mode_so);
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
  &_cmd)
{
  // This function is planned but not yet implemented.
  ROS_ERROR("The atlas/configuration handler is not implemented.\n");
/*
  for (unsigned int i = 0; i < _cmd->name.size(); ++i)
  {
    this->atlas.model->SetJointPositions();
  }
*/
}

////////////////////////////////////////////////////////////////////////////////
std::string VRCPlugin::AtlasCommandController::FindJoint(
  std::string _st1, std::string _st2)
{
  if (this->model->GetJoint(_st1))
    return _st1;
  else if (this->model->GetJoint(_st2))
    return _st2;
  else
  {
    ROS_ERROR("joint by names [%s] or [%s] not found.",
              _st1.c_str(), _st2.c_str());
    return std::string();
  }
}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::AtlasCommandController::AtlasCommandController()
{
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::InitModel(physics::ModelPtr _model)
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading AtlasCommandController since ROS hasn't been "
          << "properly initialized.  Try starting Gazebo with"
          << " ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->model = _model;

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // must match those inside AtlasPlugin
  this->jointNames.push_back(this->FindJoint("back_bkz",  "back_lbz"));
  this->jointNames.push_back(this->FindJoint("back_bky",  "back_mby"));
  this->jointNames.push_back(this->FindJoint("back_bkx",  "back_ubx"));
  this->jointNames.push_back(this->FindJoint("neck_ry",   "neck_ay"));
  this->jointNames.push_back(this->FindJoint("l_leg_hpz", "l_leg_uhz"));
  this->jointNames.push_back(this->FindJoint("l_leg_hpx", "l_leg_mhx"));
  this->jointNames.push_back(this->FindJoint("l_leg_hpy", "l_leg_lhy"));
  this->jointNames.push_back("l_leg_kny");
  this->jointNames.push_back(this->FindJoint("l_leg_aky", "l_leg_uay"));
  this->jointNames.push_back(this->FindJoint("l_leg_akx", "l_leg_lax"));
  this->jointNames.push_back(this->FindJoint("r_leg_hpz", "r_leg_uhz"));
  this->jointNames.push_back(this->FindJoint("r_leg_hpx", "r_leg_mhx"));
  this->jointNames.push_back(this->FindJoint("r_leg_hpy", "r_leg_lhy"));
  this->jointNames.push_back("r_leg_kny");
  this->jointNames.push_back(this->FindJoint("r_leg_aky", "r_leg_uay"));
  this->jointNames.push_back(this->FindJoint("r_leg_akx", "r_leg_lax"));
  this->jointNames.push_back(this->FindJoint("l_arm_shy", "l_arm_usy"));
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back(this->FindJoint("l_arm_wry", "l_arm_uwy"));
  this->jointNames.push_back(this->FindJoint("l_arm_wrx", "l_arm_mwx"));
  this->jointNames.push_back(this->FindJoint("r_arm_shy", "r_arm_usy"));
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back(this->FindJoint("r_arm_wry", "r_arm_uwy"));
  this->jointNames.push_back(this->FindJoint("r_arm_wrx", "r_arm_mwx"));

  unsigned int n = this->jointNames.size();
  this->ac.position.resize(n);
  this->ac.velocity.resize(n);
  this->ac.effort.resize(n);
  this->ac.kp_position.resize(n);
  this->ac.ki_position.resize(n);
  this->ac.kd_position.resize(n);
  this->ac.kp_velocity.resize(n);
  this->ac.i_effort_min.resize(n);
  this->ac.i_effort_max.resize(n);
  this->ac.k_effort.resize(n);

  for (unsigned int i = 0; i < n; ++i)
  {
    double val;
    this->rosNode->getParam("atlas_controller/gains/" + this->jointNames[i] +
      "/p", val);
    this->ac.kp_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + this->jointNames[i] +
      "/i", val);
    this->ac.ki_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + this->jointNames[i] +
      "/d", val);
    this->ac.kd_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + this->jointNames[i] +
      "/i_clamp", val);
    this->ac.i_effort_min[i] = -val;
    this->ac.i_effort_max[i] = val;
    this->ac.k_effort[i] =  255;

    this->ac.velocity[i]     = 0;
    this->ac.effort[i]       = 0;
    this->ac.kp_velocity[i]  = 0;
  }

  this->pubAtlasCommand =
    this->rosNode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/atlas_command", 1, true);

  this->pubAtlasSimInterfaceCommand =
    this->rosNode->advertise<atlas_msgs::AtlasSimInterfaceCommand>(
    "/atlas/atlas_sim_interface_command", 1, true);

  // ros::SubscribeOptions jointStatesSo =
  //   ros::SubscribeOptions::create<sensor_msgs::JointState>(
  //   "/atlas/joint_states", 1,
  //   boost::bind(&AtlasCommandController::GetJointStates, this, _1),
  //   ros::VoidPtr(), this->rosNode->getCallbackQueue());
  // this->subJointStates =
  //   this->rosNode->subscribe(jointStatesSo);
}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::AtlasCommandController::~AtlasCommandController()
{
  this->rosNode->shutdown();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::GetJointStates(
        const sensor_msgs::JointState::ConstPtr &_js)
{
  /// \todo: implement joint state monitoring when setting configuration
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetPIDStand(
  physics::ModelPtr atlasModel)
{
  // seated configuration
  this->ac.header.stamp = ros::Time::now();

  /*
  // StandPrep initial pose
  this->ac.position[0]  = -1.8823047867044806e-05;
  this->ac.position[1]  =  0.0016903011128306389;
  this->ac.position[2]  =  9.384587610838935e-05;
  this->ac.position[3]  =  -0.6108658313751221;
  this->ac.position[4]  =  0.30274710059165955;
  this->ac.position[5]  =  0.05022283270955086;
  this->ac.position[6]  =  -0.25109854340553284;
  this->ac.position[7]  =  0.5067367553710938;
  this->ac.position[8]  =  -0.2464604675769806;
  this->ac.position[9]  =  -0.05848940089344978;
  this->ac.position[10] =  -0.30258211493492126;
  this->ac.position[11] =  -0.07534884661436081;
  this->ac.position[12] =  -0.2539609372615814;
  this->ac.position[13] =  0.5230700969696045;
  this->ac.position[14] =  -0.2662496864795685;
  this->ac.position[15] =  0.0634056106209755;
  this->ac.position[16] =  0.29979637265205383;
  this->ac.position[17] =  -1.303655982017517;
  this->ac.position[18] =  2.000823736190796;
  this->ac.position[19] =  0.4982665777206421;
  this->ac.position[20] =  0.00030532144592143595;
  this->ac.position[21] =  -0.004383780527859926;
  this->ac.position[22] =  0.2997862696647644;
  this->ac.position[23] =  1.303290843963623;
  this->ac.position[24] =  2.0007426738739014;
  this->ac.position[25] =  -0.4982258975505829;
  this->ac.position[26] =  0.0002723461075220257;
  this->ac.position[27] =  0.004452839493751526;
  */

  // StandPrep end pose --> Stand  pose
  this->ac.position[0]  =   2.438504816382192e-05;
  this->ac.position[1]  =   0.0015186156379058957;
  this->ac.position[2]  =   9.983908967114985e-06;
  this->ac.position[3]  =   -0.0010675729718059301;
  this->ac.position[4]  =   -0.0003740221436601132;
  this->ac.position[5]  =   0.06201673671603203;
  this->ac.position[6]  =  -0.2333149015903473;
  this->ac.position[7]  =   0.5181407332420349;
  this->ac.position[8]  =  -0.27610817551612854;
  this->ac.position[9]  =   -0.062101610004901886;
  this->ac.position[10] =  0.00035181696875952184;
  this->ac.position[11] =   -0.06218484416604042;
  this->ac.position[12] =  -0.2332201600074768;
  this->ac.position[13] =   0.51811283826828;
  this->ac.position[14] =  -0.2762000858783722;
  this->ac.position[15] =   0.06211360543966293;
  this->ac.position[16] =   0.29983898997306824;
  this->ac.position[17] =   -1.303462266921997;
  this->ac.position[18] =   2.0007927417755127;
  this->ac.position[19] =   0.49823325872421265;
  this->ac.position[20] =  0.0003098883025813848;
  this->ac.position[21] =   -0.0044272784143686295;
  this->ac.position[22] =   0.29982614517211914;
  this->ac.position[23] =   1.3034454584121704;
  this->ac.position[24] =   2.000779867172241;
  this->ac.position[25] =  -0.498238742351532;
  this->ac.position[26] =  0.0003156556049361825;
  this->ac.position[27] =   0.004448802210390568;


  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->ac.k_effort[i] =  255;

  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(atlasModel->GetName() + "::" +
                              this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIFREEZE()
{
  atlas_msgs::AtlasSimInterfaceCommand ac;
  ac.header.stamp = ros::Time::now();
  ac.behavior = ac.FREEZE;
  this->pubAtlasSimInterfaceCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIStandPrep()
{
  atlas_msgs::AtlasSimInterfaceCommand ac;
  ac.header.stamp = ros::Time::now();
  ac.behavior = ac.STAND_PREP;
  ac.k_effort.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->ac.k_effort[i] =  0;
  this->pubAtlasSimInterfaceCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIStand()
{
  atlas_msgs::AtlasSimInterfaceCommand ac;
  ac.k_effort.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    ac.k_effort[i] =  0;
  ac.header.stamp = ros::Time::now();
  ac.behavior = ac.STAND;
  this->pubAtlasSimInterfaceCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetSeatingConfiguration(
  physics::ModelPtr atlasModel)
{
  // seated configuration
  this->ac.header.stamp = ros::Time::now();
  this->ac.position[0]  =   0.00;
  this->ac.position[1]  =   0.00;
  this->ac.position[2]  =   0.00;
  this->ac.position[3]  =   0.00;
  this->ac.position[4]  =   0.45;
  this->ac.position[5]  =   0.00;
  this->ac.position[6]  =  -1.60;
  this->ac.position[7]  =   1.60;
  this->ac.position[8]  =  -0.10;
  this->ac.position[9]  =   0.00;
  this->ac.position[10] =  -0.45;
  this->ac.position[11] =   0.00;
  this->ac.position[12] =  -1.60;
  this->ac.position[13] =   1.60;
  this->ac.position[14] =  -0.10;
  this->ac.position[15] =   0.00;
  this->ac.position[16] =   0.00;
  this->ac.position[17] =   0.00;
  this->ac.position[18] =   1.50;
  this->ac.position[19] =   1.50;
  this->ac.position[20] =  -3.00;
  this->ac.position[21] =   0.00;
  this->ac.position[22] =   0.00;
  this->ac.position[23] =   0.00;
  this->ac.position[24] =   1.50;
  this->ac.position[25] =  -1.50;
  this->ac.position[26] =  -3.00;
  this->ac.position[27] =   0.00;

  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(atlasModel->GetName() + "::" +
                              this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetStandingConfiguration(
  physics::ModelPtr atlasModel)
{
  // standing configuration
  this->ac.header.stamp = ros::Time::now();
  this->ac.position[0]  =   0.00;
  this->ac.position[1]  =   0.00;
  this->ac.position[2]  =   0.00;
  this->ac.position[3]  =   0.00;
  this->ac.position[4]  =   0.00;
  this->ac.position[5]  =   0.00;
  this->ac.position[6]  =   0.00;
  this->ac.position[7]  =   0.00;
  this->ac.position[8]  =   0.00;
  this->ac.position[9]  =   0.00;
  this->ac.position[10] =   0.00;
  this->ac.position[11] =   0.00;
  this->ac.position[12] =   0.00;
  this->ac.position[13] =   0.00;
  this->ac.position[14] =   0.00;
  this->ac.position[15] =   0.00;
  this->ac.position[16] =   0.00;
  this->ac.position[17] =  -1.60;
  this->ac.position[18] =   0.00;
  this->ac.position[19] =   0.00;
  this->ac.position[20] =   0.00;
  this->ac.position[21] =   0.00;
  this->ac.position[22] =   0.00;
  this->ac.position[23] =   1.60;
  this->ac.position[24] =   0.00;
  this->ac.position[25] =   0.00;
  this->ac.position[26] =   0.00;
  this->ac.position[27] =   0.00;

  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(atlasModel->GetName() + "::" +
                              this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}
}
