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

#include <angles/angles.h>
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
  this->rosNode = NULL;
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

  std::string cmdVelTimeout = "cmd_vel_timeout";
  if (this->rosNode->getParam(cmdVelTimeout, this->cmdVelTopicTimeout))
  {
    ROS_INFO("atlas fake walk teleop command timeout set to %f seconds.",
             this->cmdVelTopicTimeout);
  }
  else
  {
    ROS_INFO("atlas fake walk teleop command timeout param not set, "
             "defaults to 0.1 seconds.\n");
    this->cmdVelTopicTimeout = 0.1;
  }

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&VRCPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::PinAtlas(bool _with_gravity)
{
  // pinning robot, potentially turning off effect of gravity
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

  this->atlas.model->SetGravityMode(_with_gravity);

  this->SetFeetCollide("none");
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::UnpinAtlas()
{
  // nominal
  this->warpRobotWithCmdVel = false;
  this->atlas.model->SetGravityMode(true);
  if (this->atlas.pinJoint)
    this->RemoveJoint(this->atlas.pinJoint);
  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);
  this->SetFeetCollide("all");

  if (this->world->GetPhysicsEngine()->GetType() == "simbody" ||
      this->world->GetPhysicsEngine()->GetType() == "dart")
  {
    // simulate un-freezing simbody or dart unlock free joints
    // Currently we do this to all the links in the model,
    // but ideally we can do this to only the link(s) with
    // a free 6-dof mobilizer.
    physics::Link_V links = this->atlas.model->GetLinks();
    for(physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
      (*li)->SetLinkStatic(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetFeetCollide(const std::string &_mode)
{
  physics::LinkPtr l_foot_link = this->atlas.model->GetLink("l_foot");
  if (!l_foot_link)
    ROS_WARN("Couldn't find l_foot link when setting collide mode");
  else
    l_foot_link->SetCollideMode(_mode);

  physics::LinkPtr r_foot_link = this->atlas.model->GetLink("r_foot");
  if (!r_foot_link)
    ROS_WARN("Couldn't find r_foot link when setting collide mode");
  else
    r_foot_link->SetCollideMode(_mode);
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
    this->atlas.model->SetGravityMode(false);
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;

    this->atlas.model->SetGravityMode(false);
    this->atlas.model->GetLink("l_foot")->SetGravityMode(true);
    this->atlas.model->GetLink("r_foot")->SetGravityMode(true);

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

    double distBelow = 0.0;
    physics::EntityPtr entityBelow;
    physics::EntityPtr fromEntity = this->atlas.pinLink;
    std::string objectBelow;
    fromEntity->GetNearestEntityBelow(distBelow, objectBelow);
    entityBelow = this->world->GetEntity(objectBelow);
    gzdbg << fromEntity->GetName() << " "
          << distBelow << " " << objectBelow << "\n";
    // if entity below is part of atlas, set it to fromEntity
    // and keep searching below
    while (entityBelow && (entityBelow->GetParentModel() ==
      fromEntity->GetParentModel()))
    {
      objectBelow.clear();
      fromEntity = entityBelow;
      fromEntity->GetNearestEntityBelow(distBelow, objectBelow);
      entityBelow = this->world->GetEntity(objectBelow);
      gzdbg << fromEntity->GetName() << " "
            << distBelow << " " << objectBelow << "\n";
    }
    if (entityBelow && fromEntity)
    {
      // gzdbg << objectBelow << "\n";
      // gzdbg << groundHeight << "\n";
      // gzdbg << groundBB.max.z << "\n";
      // gzdbg << groundBB.min.z << "\n";

      // slightly above ground and upright
      // fromEntity->GetCollisionBoundingBox().min.z gives us the
      // lowest point of atlas robot. Set pin location to 1.15m
      // above it.
      atlasPose.pos.z = fromEntity->GetCollisionBoundingBox().min.z -
        distBelow + 1.15;
    }
    else
    {
      gzwarn << "No entity below robot, or GetEntityBelowPoint "
             << "returned NULL pointer. Assume ground height = 0.0m\n";
      // put atlas back
      atlasPose.pos.z =  1.15;
    }

    // set robot pose and pin it
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

    this->world->SetPaused(paused);
  }
  else if (_str == "pinned")
  {
    this->PinAtlas(false);
  }
  else if (_str == "pinned_with_gravity")
  {
    this->PinAtlas(true);
  }
  else if (_str == "nominal")
  {
    this->UnpinAtlas();
  }
  else if (_str == "pid_stand")
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
    this->atlas.model->SetGravityMode(false);

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

void VRCPlugin::StepDataToTwist(
  const atlas_msgs::AtlasBehaviorStepData & _step,
  double _dt,
  geometry_msgs::Twist::Ptr _twist)
{
  // Which foot are we placing last?
  unsigned int foot_idx = _step.foot_index;
  // Where's the pelvis (which we'll pin) with respect to the foot
  // that we're placing last?
  math::Pose current_pelvis_pose = this->atlas.pinLink->GetWorldPose();
  // And where is the foot?
  // I'm pretty sure that 0=left and 1=right, but I can't find
  // documentation on that.
  physics::LinkPtr foot_link = this->atlas.model->GetLink(
    (foot_idx == 0) ? "l_foot" : "r_foot");
  if (!foot_link)
  {
    ROS_ERROR("Couldn't find Atlas's foot link when faking walking.");
    return;
  }
  math::Pose current_foot_pose = foot_link->GetWorldPose();
  math::Pose T_foot_pelvis = current_pelvis_pose - current_foot_pose;
  ROS_DEBUG("Current foot pose: %f %f %f",
    current_foot_pose.pos.x,
    current_foot_pose.pos.y,
    current_foot_pose.pos.z);

  // Convert from ROS geometry_msgs/Pose message to Gazebo math::Pose
  geometry_msgs::Pose tmp_pose = _step.pose;
  math::Pose goal_foot_pose;
  goal_foot_pose.pos.x = tmp_pose.position.x;
  goal_foot_pose.pos.y = tmp_pose.position.y;
  goal_foot_pose.pos.z = tmp_pose.position.z;
  goal_foot_pose.rot.w = tmp_pose.orientation.w;
  goal_foot_pose.rot.x = tmp_pose.orientation.x;
  goal_foot_pose.rot.y = tmp_pose.orientation.y;
  goal_foot_pose.rot.z = tmp_pose.orientation.z;

  ROS_DEBUG("Goal foot pose: %f %f %f",
    goal_foot_pose.pos.x,
    goal_foot_pose.pos.y,
    goal_foot_pose.pos.z);

  // Where should the pelvis be to achieve the desired foot pose
  // (assuming that the robot configuration doesn't change)?
  math::Pose goal_pelvis_pose = goal_foot_pose + T_foot_pelvis;

  // How far do we need to move in the plane to get there?
  double dx = goal_pelvis_pose.pos.x - current_pelvis_pose.pos.x;
  double dy = goal_pelvis_pose.pos.y - current_pelvis_pose.pos.y;
  double dyaw = angles::shortest_angular_distance(
    current_pelvis_pose.rot.GetAsEuler().z,
    goal_pelvis_pose.rot.GetAsEuler().z);

  // Transform into ego-centric frame, which is how the resulting velocities
  // will be interpreted.
  math::Vector3 local_d(dx, dy, 0);
  local_d = current_pelvis_pose.rot.RotateVectorReverse(local_d);

  // Build a cmd_vel message
  _twist->linear.x = local_d.x / _dt;
  _twist->linear.y = local_d.y / _dt;
  _twist->linear.z = 0.0;
  _twist->angular.x = 0.0;
  _twist->angular.y = 0.0;
  _twist->angular.z = dyaw / _dt;

  ROS_DEBUG("Current pose: %f %f %f",
    current_pelvis_pose.pos.x,
    current_pelvis_pose.pos.y,
    current_pelvis_pose.rot.GetAsEuler().z);
  ROS_DEBUG("Goal pose: %f %f %f",
    goal_pelvis_pose.pos.x,
    goal_pelvis_pose.pos.y,
    goal_pelvis_pose.rot.GetAsEuler().z);
  ROS_DEBUG("Computed velocity (dt=%f): %f %f %f",
    _dt, _twist->linear.x, _twist->linear.y, _twist->angular.z);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetFakeASIC(
  const atlas_msgs::AtlasSimInterfaceCommand::ConstPtr &_asic)
{
  // Disable the real BDI behavior library
  atlas_msgs::AtlasSimInterfaceCommand ac;
  ac.header.stamp = ros::Time::now();
  ac.behavior = ac.USER;
  for (size_t i=0; i<this->atlasCommandController.jointNames.size(); i++)
    ac.k_effort.push_back(255);
  this->atlasCommandController.pubAtlasSimInterfaceCommand.publish(ac);

  geometry_msgs::Twist::Ptr zero_vel(new geometry_msgs::Twist);
  if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::STAND)
  {
    // We fake STAND by pinning the robot.
    this->PinAtlas(true);
    this->SetRobotCmdVel(zero_vel, 0.0);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::USER)
  {
    this->UnpinAtlas();
    this->SetRobotCmdVel(zero_vel, 0.0);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::FREEZE)
  {
    // We fake FREEZE by doing PID around current joint positions.
    if (!this->atlasCommandController.js_valid)
    {
      ROS_WARN("FREEZE commanded, but no valid joint state yet,"
               "so I can't set PID position goals.");
      return;
    }
    ROS_ASSERT(this->atlasCommandController.js->position.size() ==
      this->atlasCommandController.ac.position.size());
    for (size_t i=0; i < this->atlasCommandController.js->position.size(); i++)
    {
      // Here we just set desired positions.
      // We assume that everything else in
      // this->atlasCommandController->ac was set properly in
      // VRCPlugin::AtlasCommandController::InitModel().
      this->atlasCommandController.ac.k_effort[i] = 255;
      this->atlasCommandController.ac.position[i] =
        this->atlasCommandController.js->position[i];
    }
    this->atlasCommandController.pubAtlasCommand.publish(
      this->atlasCommandController.ac);
    this->UnpinAtlas();
    this->SetRobotCmdVel(zero_vel, 0.0);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP)
  {
    // no-op
    this->SetRobotCmdVel(zero_vel, 0.0);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::WALK)
  {
    if (_asic->walk_params.use_demo_walk)
    {
      ROS_WARN("Demo walk requested, but it's unsupported.");
      return;
    }
    // We fake WALK by turning it into a planar velocity and passing it to
    // the existing fake teleop system.

    // How much time should we take?
    double dt = 0.0;
    for (size_t i=0; i<_asic->walk_params.step_queue.size(); i++)
    {
      dt += _asic->walk_params.step_queue[i].duration;
    }
    size_t step_idx = _asic->walk_params.step_queue.size()-1;
    geometry_msgs::Twist::Ptr cmd_vel(new geometry_msgs::Twist);
    this->StepDataToTwist(_asic->walk_params.step_queue[step_idx],
                          dt, cmd_vel);
    this->atlas.currentStepIndex = _asic->walk_params.step_queue[0].step_index;
    this->atlas.lastStepIndex =
      _asic->walk_params.step_queue[step_idx].step_index;
    this->SetFeetCollide("none");
    this->SetRobotCmdVel(cmd_vel, dt);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::STEP)
  {
    if (_asic->step_params.use_demo_walk)
    {
      ROS_WARN("Demo walk requested, but it's unsupported.");
      return;
    }
    // We fake STEP by turning it into a planar velocity and passing it to
    // the existing fake teleop system.

    // How much time should we take?
    double dt = _asic->step_params.desired_step.duration;
    geometry_msgs::Twist::Ptr cmd_vel(new geometry_msgs::Twist);
    this->StepDataToTwist(_asic->step_params.desired_step, dt, cmd_vel);
    this->atlas.currentStepIndex = _asic->step_params.desired_step.step_index;
    this->atlas.lastStepIndex = _asic->step_params.desired_step.step_index;
    this->SetFeetCollide("none");
    this->SetRobotCmdVel(cmd_vel, dt);
  }
  else if (_asic->behavior == atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE)
  {
    // We fake STAND by pinning the robot.
    this->PinAtlas(true);
    this->SetRobotCmdVel(zero_vel, 0.0);
  }
  else
  {
    ROS_WARN("SetFakeASIC: ignoring unknown behavior type %u",
      _asic->behavior);
    return;
  }
  this->atlas.currentBehavior = _asic->behavior;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVelTopic(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  // hard code timeout to 0.1 seconds
  this->SetRobotCmdVel(_cmd, this->cmdVelTopicTimeout);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd,
                               double _duration)
{
  if (_duration > 0.0)
    this->warpRobotStopTime = this->world->GetSimTime() + _duration;
  else
    this->warpRobotStopTime = common::Time(0,0);

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
  physics::JointPtr joint;
  if (_world->GetPhysicsEngine()->GetType() == "ode" ||
      _world->GetPhysicsEngine()->GetType() == "bullet")
  {
    joint = _world->GetPhysicsEngine()->CreateJoint(
      _type, _model);
    joint->Attach(_link1, _link2);
    // load adds the joint to a vector of shared pointers kept
    // in parent and child links, preventing joint from being destroyed.
    joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
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
  }
  else if (_world->GetPhysicsEngine()->GetType() == "simbody" ||
           _world->GetPhysicsEngine()->GetType() == "dart")
  {
    // simulate freezing lock simbody or dart free joints
    // Currently we do this to all the links in the model,
    // but ideally we can do this to only the link(s) with
    // a free 6-dof mobilizer.
    physics::Link_V links = _model->GetLinks();
    for(physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
      (*li)->SetLinkStatic(true);
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

  math::Vector3 atlasVehicleRelPos = math::Vector3(-0.06, 0.3, 1.28);

  // hardcoded offset of the robot when it's seated in the vehicle driver seat.
  this->atlas.vehicleRelPose = math::Pose(atlasVehicleRelPos,
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
  this->atlas.vehicleRelPose = math::Pose(atlasVehicleRelPos,
      math::Quaternion());

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
  //this->atlasCommandController.SetStandingConfiguration(this->atlas.model);
  this->atlasCommandController.SetPIDStand(this->atlas.model);
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
  // At t-t0 < startupStandPrepDuration seconds, pinned.
  // At t-t0 = startupStandPrepDuration seconds, begin StandPrep mode.
  // At t-t0 = startupNominal seconds, unpinned, nominal.
  // At t-t0 = startupStand seconds, start Stand mode.
  if (this->atlas.startupSequence == Robot::NONE)
  {
    // Load and Spawn Robot
    this->atlas.InsertModel(this->world, this->sdf);
  }
  else if (this->atlas.startupSequence == Robot::SPAWN_QUEUED)
  {
    if (this->atlas.CheckGetModel(this->world))
    {
      this->atlas.startupSequence = Robot::SPAWN_SUCCESS;
    }
    else
    {
      // still waiting for robot to be spawned
      ROS_INFO("waiting for atlas robot to be spawned.");
    }
  }
  else if (this->atlas.startupSequence == Robot::SPAWN_SUCCESS)
  {
    // initialize Atlas Command Controller
    // Advertise ros topics "atlas/atlas_command" and
    // "atlas/atlas_sim_interface_command". Subscribe to
    // "atlas/joint_states".
    ROS_INFO("spawn success, set pinLink and call initialize controller");

    this->atlas.pinLink = this->atlas.model->GetLink(this->atlas.pinLinkName);

    if (!this->atlas.pinLink)
    {
      ROS_ERROR("atlas robot pin link not found, VRCPlugin will not work.");
      this->atlas.startupSequence = Robot::NONE;
      return;
    }

    // Note: hardcoded link by name: @todo: make this a pugin param
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    // initialize atlas command controller
    this->atlasCommandController.InitModel(this->atlas.model);

    this->atlas.startupSequence = Robot::INIT_MODEL_SUCCESS;
  }
  else if (this->atlas.startupSequence == Robot::INIT_MODEL_SUCCESS)
  {
    // robot could have 2 distinct startup modes in sim:  bdi_stand | pinned
    // bdi_stand:
    //   Sets robot startup configuration to that of bdi stand behavior.
    //   PID robot at stand configuration for a few seconds.
    //   Turn on BDI controller, dynamically balance the robot.
    // pinned:
    //   Robot PID's to zero joint angles, and pinned to the world.
    //   If StartupHarnessDuration > 0 unpin the robot after duration.

    std::string startInVehicleName = "robot_start_in_vehicle";
    bool startInVehicle = false;
    if (this->rosNode->getParam(startInVehicleName, startInVehicle) &&
                                                            startInVehicle)
    {
      gzdbg << "Starting robot in vehicle." << std::endl;
      geometry_msgs::Pose::Ptr poseMsg(new geometry_msgs::Pose());
      this->RobotEnterCar(poseMsg);
      this->atlas.startupSequence = Robot::INITIALIZED;
    }
    else if (atlas.startupMode == "bdi_stand")
    {
      switch (this->atlas.bdiStandSequence)
      {
        case Robot::BS_NONE:
        {
          // ROS_INFO("BS_NONE");
          this->SetRobotMode("pid_stand");
          // start the rest of the sequence
          this->atlas.bdiStandSequence = Robot::BS_PID_PINNED;
          this->atlas.startupBDIStandStartTime = this->world->GetSimTime();
          break;
        }
        case Robot::BS_PID_PINNED:
        {
          // ROS_INFO("BS_PID_PINNED");
          if ((curTime - this->atlas.startupBDIStandStartTime.Double()) >
            atlas.startupStandPrepDuration)
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
            atlas.startupNominal)
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
              atlas.startupStand)
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
            ROS_INFO("Atlas will stay pinned.");
            this->atlas.pinnedSequence = Robot::PS_INITIALIZED;
          }
          else
          {
            ROS_INFO("Resume to nominal mode after %f seconds.",
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

    if (this->warpRobotWithCmdVel &&
        (this->world->GetSimTime() <= this->warpRobotStopTime))
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

  if ((this->atlas.startupSequence == Robot::INITIALIZED) &&
      this->cheatsEnabled)
  {
    // publish fake AtlasSimInterfaceState via this->pubFakeASIS
    atlas_msgs::AtlasSimInterfaceState asis;
    asis.error_code = atlas_msgs::AtlasSimInterfaceState::NO_ERRORS;
    asis.current_behavior = this->atlas.currentBehavior;
    asis.desired_behavior = this->atlas.currentBehavior;
    for (size_t i=0; i<asis.f_out.size(); i++)
      asis.f_out[i] = 0.0;
    math::Pose cur_pose = this->atlas.pinLink->GetWorldPose();
    asis.pos_est.position.x = cur_pose.pos.x;
    asis.pos_est.position.y = cur_pose.pos.y;
    asis.pos_est.position.z = cur_pose.pos.z;
    math::Vector3 cur_vel = this->atlas.pinLink->GetWorldLinearVel();
    asis.pos_est.velocity.x = cur_vel.x;
    asis.pos_est.velocity.y = cur_vel.y;
    asis.pos_est.velocity.z = cur_vel.z;
    physics::LinkPtr l_foot_link = this->atlas.model->GetLink("l_foot");
    if (!l_foot_link)
      ROS_WARN("Couldn't find l_foot link when publishing fake behavior data.");
    else
    {
      math::Pose l_foot_pose = l_foot_link->GetWorldPose();
      asis.foot_pos_est[0].position.x = l_foot_pose.pos.x;
      asis.foot_pos_est[0].position.y = l_foot_pose.pos.y;
      asis.foot_pos_est[0].position.z = l_foot_pose.pos.z;
      asis.foot_pos_est[0].orientation.w = l_foot_pose.rot.w;
      asis.foot_pos_est[0].orientation.x = l_foot_pose.rot.x;
      asis.foot_pos_est[0].orientation.y = l_foot_pose.rot.y;
      asis.foot_pos_est[0].orientation.z = l_foot_pose.rot.z;
    }
    physics::LinkPtr r_foot_link = this->atlas.model->GetLink("r_foot");
    if (!l_foot_link)
      ROS_WARN("Couldn't find l_foot link when publishing fake behavior data.");
    else
    {
      math::Pose r_foot_pose = r_foot_link->GetWorldPose();
      asis.foot_pos_est[1].position.x = r_foot_pose.pos.x;
      asis.foot_pos_est[1].position.y = r_foot_pose.pos.y;
      asis.foot_pos_est[1].position.z = r_foot_pose.pos.z;
      asis.foot_pos_est[1].orientation.w = r_foot_pose.rot.w;
      asis.foot_pos_est[1].orientation.x = r_foot_pose.rot.x;
      asis.foot_pos_est[1].orientation.y = r_foot_pose.rot.y;
      asis.foot_pos_est[1].orientation.z = r_foot_pose.rot.z;
    }
    for (size_t i=0; i<asis.k_effort.size(); i++)
      asis.k_effort[i] = 0;

    // Do what we can for the behavior-specific feedback data
    if (asis.current_behavior == atlas_msgs::AtlasSimInterfaceCommand::WALK)
    {
      double time_remaining =
        (this->warpRobotStopTime - this->world->GetSimTime()).Double();
      if (time_remaining > 0.0)
      {
        // Assuming that t_step_rem should be in milliseconds
        asis.walk_feedback.t_step_rem = time_remaining * 1e3;
        asis.walk_feedback.current_step_index = this->atlas.currentStepIndex;
      }
      else
      {
        asis.walk_feedback.t_step_rem = 0.0;
        asis.walk_feedback.current_step_index = this->atlas.lastStepIndex;
      }
      asis.walk_feedback.next_step_index_needed = this->atlas.lastStepIndex+1;
      //asis.walk_feedback.status_flags
      //asis.walk_feedback.step_queue_saturated
    }

    this->atlas.pubFakeASIS.publish(asis);
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
    ROS_INFO("VRCPlugin: fire_hose_model [%s] not found, threading disabled.",
      fireHoseModelName.c_str());
    return;
  }
  this->initialFireHosePose = this->fireHoseModel->GetWorldPose();

  // Get coupling link
  std::string couplingLinkName = sdf->Get<std::string>("coupling_link");
  this->couplingLink = this->fireHoseModel->GetLink(couplingLinkName);
  if (!this->couplingLink)
  {
    ROS_INFO("VRCPlugin: coupling link [%s] not found, threading disabled.",
      couplingLinkName.c_str());
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
    ROS_ERROR("VRCPlugin: standpipe model [%s] not found",
      standpipeModelName.c_str());
    return;
  }

  // Get spout link
  std::string spoutLinkName = sdf->Get<std::string>("spout_link");
  this->spoutLink = this->standpipeModel->GetLink(spoutLinkName);
  if (!this->spoutLink)
  {
    ROS_ERROR("VRCPlugin: spout link [%s] not found",
      spoutLinkName.c_str());
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
    ROS_WARN("VRCPlugin: valve model [%s] not found, scoring will be wrong",
      valveModelName.c_str());
  }
  else
  {
    std::string valveJointName;
    if (sdf->HasElement("valve_joint"))
      valveJointName = sdf->Get<std::string>("valve_joint");
    else
      valveJointName = "valve";
    this->valveJoint = this->valveModel->GetJoint(valveJointName);
    if (!this->valveJoint)
    {
      ROS_WARN("VRCPlugin: valve joint [%s] not found, scoring will be wrong",
        valveJointName.c_str());
    }
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
#if GAZEBO_MAJOR_VERSION >= 4
    this->fireHoseJoints[i]->SetPosition(0u, 0.0);
#else
    this->fireHoseJoints[i]->SetAngle(0u, 0.0);
#endif
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
  double valveAng = 0;
  if (this->drcFireHose.valveJoint)
    valveAng = this->drcFireHose.valveJoint->GetAngle(0).Radian();

  /* uncomment for debugging
  gzdbg << " connectPose [" << connectPose
        << "] [" << connectPose.rot.GetXAxis()
        << "] [" << connectPose.rot.GetYAxis()
        << "] [" << connectPose.rot.GetZAxis() << "]\n";
  gzerr << "<coupling_relative_pose>"
        << relativePose + math::Pose(0, 0, collisionSurfaceZOffset, 0, 0, 0)
        << "</coupling_relative_pose>\n";
  gzdbg << "relativePose axis [" << relativePose.rot.GetXAxis()  // bingo
        << "] [" << relativePose.rot.GetYAxis()
        << "] [" << relativePose.rot.GetZAxis() << "]\n";
  gzdbg << "offset [" << relativePose - connectPose << "]\n";
  gzwarn << "insert [" << posErrInsert
         << "] center [" << posErrCenter
         << "] rpy [" << rotErr
         << "] valve [" << valveAng
         << "]\n";
  */

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

      this->drcFireHose.screwJoint->SetParam("thread_pitch", 0,
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
 : currentBehavior(-1)
{
  this->startupSequence = Robot::NONE;
  this->bdiStandSequence = Robot::BS_NONE;
  this->pinnedSequence = Robot::PS_NONE;

  // bunch of hardcoded presets
  this->startupHarnessDuration = 5;
  this->startupStandPrepDuration = 2.0;
  this->startupNominal = this->startupStandPrepDuration + 2.0;
  this->startupStand = this->startupNominal + 0.1;

}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::Robot::~Robot()
{
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Robot::InsertModel(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
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
      this->startupSequence = Robot::NONE;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
bool VRCPlugin::Robot::CheckGetModel(physics::WorldPtr _world)
{
  this->model = _world->GetModel(this->modelName);
  if (this->model)
  {
    // set initial pose of Atlas based on
    // ros params "robot_initial_pose/[x|y|z|roll|pitch|yaw]"
    this->model->SetInitialRelativePose(this->spawnPose);
    this->model->SetWorldPose(this->spawnPose);

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
    ROS_INFO("atlas/time_to_unpin not specified, default harness duration to"
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
      boost::bind(&VRCPlugin::SetRobotCmdVelTopic, this, _1),
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

    std::string fake_asic_topic_name = "atlas/fake/atlas_sim_interface_command";
    ros::SubscribeOptions fake_asic_so =
      ros::SubscribeOptions::create<atlas_msgs::AtlasSimInterfaceCommand>(
      fake_asic_topic_name, 100,
      boost::bind(&VRCPlugin::SetFakeASIC, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subFakeASIC = this->rosNode->subscribe(fake_asic_so);

    // ros advertisement
    this->atlas.pubFakeASIS =
      this->rosNode->advertise<atlas_msgs::AtlasSimInterfaceState>(
      "atlas/fake/atlas_sim_interface_state", 1, true);
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
    ROS_INFO("VRCPlugin: joint by names [%s] or [%s] not found.",
              _st1.c_str(), _st2.c_str());
    return std::string();
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string VRCPlugin::AtlasCommandController::FindJoint(
  std::string _st1, std::string _st2, std::string _st3)
{
  return this->FindJoint(this->FindJoint(_st1, _st2), _st3);
}

////////////////////////////////////////////////////////////////////////////////
VRCPlugin::AtlasCommandController::AtlasCommandController()
 : js_valid(false)
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

  // Get atlas version, and set joint count
  this->atlasVersion = 5;
  if (!this->rosNode->getParam("atlas_version", this->atlasVersion))
  {
    ROS_WARN("atlas_version not set, assuming version 5");
  }

  // Read the subversion of Atlas. The parameter is optional
  this->atlasSubVersion = 0;
  this->rosNode->getParam("atlas_sub_version", this->atlasSubVersion);

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
  this->jointNames.push_back(this->FindJoint("l_arm_shz", "l_arm_shy", "l_arm_usy"));
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back(this->FindJoint("l_arm_wry", "l_arm_uwy"));
  this->jointNames.push_back(this->FindJoint("l_arm_wrx", "l_arm_mwx"));

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->jointNames.push_back(this->FindJoint("l_arm_wry2", "l_arm_lwy"));
  }

  this->jointNames.push_back(
      this->FindJoint("r_arm_shz", "r_arm_shy", "r_arm_usy"));
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back(this->FindJoint("r_arm_wry", "r_arm_uwy"));
  this->jointNames.push_back(this->FindJoint("r_arm_wrx", "r_arm_mwx"));

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->jointNames.push_back(this->FindJoint("r_arm_wry2", "r_arm_lwy"));
  }

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
    "atlas/atlas_command", 1, true);

  this->pubAtlasSimInterfaceCommand =
    this->rosNode->advertise<atlas_msgs::AtlasSimInterfaceCommand>(
    "atlas/atlas_sim_interface_command", 1, true);

  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "atlas/joint_states", 1,
    boost::bind(&AtlasCommandController::GetJointStates, this, _1),
    ros::VoidPtr(), this->rosNode->getCallbackQueue());
  this->subJointStates = this->rosNode->subscribe(jointStatesSo);
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
  this->js = _js;
  this->js_valid = true;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetPIDStand(
  physics::ModelPtr atlasModel)
{
  // seated configuration
  this->ac.header.stamp = ros::Time::now();

  int index = 0;

  if (this->atlasVersion < 4)
  {
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
  }
  else
  {
    // StandPrep end pose --> Stand  pose
    this->ac.position[index++]  =  0.0; //back_bkz
    this->ac.position[index++]  =  0.00225254; //back_bkz
    this->ac.position[index++]  =  0.0; //back_bkx
    this->ac.position[index++]  =  -0.1106; //neck_ry

    this->ac.position[index++]  =  -0.00692196; //l_hpz
    this->ac.position[index++]  =  0.0690; //l_hpx
    this->ac.position[index++]  = -0.472917;    // l_hpy
    this->ac.position[index++]  =  0.93299556; //l_kny
    this->ac.position[index++]  = -0.4400587703;   // l_aky
    this->ac.position[index++]  = -0.0689798; //l_akx

    this->ac.position[index++] = -this->ac.position[4]; //r_hpz
    this->ac.position[index++] = -this->ac.position[5]; //r_hpx
    this->ac.position[index++] = this->ac.position[6]; //r_hpy
    this->ac.position[index++] = this->ac.position[7]; //r_kny
    this->ac.position[index++] = this->ac.position[8]; //r_aky
    this->ac.position[index++] = -this->ac.position[9]; //r_akx

    this->ac.position[index++] =  -0.299681926;  // l_shy || shz
    this->ac.position[index++] =  -1.300665; //l_shx
    this->ac.position[index++] =  1.852762; //l_ely
    this->ac.position[index++] =  0.492914; //l_elx
    this->ac.position[index++] =  0.00165999; //l_wry
    this->ac.position[index++] =  -0.00095767089; //l_wrx

    // Atlas version 4.1 has no wry2 joints
    if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
        this->atlasVersion > 4)
    {
      // l_arm_wry2
      this->ac.position[index++] =  0.01305307;
    }

    this->ac.position[index++] =  (this->atlasVersion >= 4) ?
      -this->ac.position[16] : this->ac.position[16]; //r_arm_shz

    this->ac.position[index++] =  -this->ac.position[17]; //r_arm_shx
    this->ac.position[index++] =  this->ac.position[18]; //r_arm_ely
    this->ac.position[index++] =  -this->ac.position[19]; //r_arm_elx
    this->ac.position[index++] =  this->ac.position[20]; //r_arm_wry
    this->ac.position[index++] =  -this->ac.position[21]; //r_arm_wrx

    // Atlas version 4.1 has no wry2 joints
    if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
        this->atlasVersion > 4)
    {
      // r_arm_wry2
      this->ac.position[index++] = this->ac.position[22];
    }

    this->ac.effort[1] = -27.6;
    this->ac.effort[6] = -23.5;
    this->ac.effort[7] = -105.7;
    this->ac.effort[8] = 24.1;
    this->ac.effort[6+6] = -23.5;
    this->ac.effort[7+6] = -105.7;
    this->ac.effort[8+6] = 24.1;
  }


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
    this->ac.k_effort[i] = 0;
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
  int index = 0;

  // seated configuration
  this->ac.header.stamp = ros::Time::now();
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++]  =   0.45;
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++]  =  -1.60;
  this->ac.position[index++]  =   1.60;
  this->ac.position[index++]  =  -0.10;
  this->ac.position[index++]  =   0.00;
  this->ac.position[index++] =  -0.45;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =  -1.60;
  this->ac.position[index++] =   1.60;
  this->ac.position[index++] =  -0.10;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   1.50;
  this->ac.position[index++] =   1.50;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->ac.position[index++] = 0.0;
  }

  this->ac.position[index++] = -this->ac.position[16];
  this->ac.position[index++] = -this->ac.position[17];
  this->ac.position[index++] = this->ac.position[18];
  this->ac.position[index++] = -this->ac.position[19];
  this->ac.position[index++] = this->ac.position[20];
  this->ac.position[index++] = -this->ac.position[21];

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->ac.position[index++] = this->ac.position[22];
  }

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
  int index = 0;

  // standing configuration
  this->ac.header.stamp = ros::Time::now();
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =  -1.60;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;
  this->ac.position[index++] =   0.00;

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->ac.position[index++] =   0.00;
  }

  this->ac.position[index++] = -this->ac.position[16];
  this->ac.position[index++] = -this->ac.position[17];
  this->ac.position[index++] = this->ac.position[18];
  this->ac.position[index++] = -this->ac.position[19];
  this->ac.position[index++] = this->ac.position[20];
  this->ac.position[index++] = -this->ac.position[21];

  // Atlas version 4.1 has no wry2 joints
  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
  {
    this->ac.position[index++] = this->ac.position[22];
  }

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
