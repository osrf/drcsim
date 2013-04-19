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

#include "VRCPlugin.h"

namespace gazebo
{
GZ_REGISTER_WORLD_PLUGIN(VRCPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
VRCPlugin::VRCPlugin()
{
  /// initial anchor pose
  this->warpRobotWithCmdVel = false;
  this->bdiStandPrep = false;
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

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&VRCPlugin::DeferredLoad, this));
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

  // Load Robot
  this->atlas.Load(this->world, this->sdf);

  // Load Vehicle
  this->drcVehicle.Load(this->world, this->sdf);

  // Load fire hose and standpipe
  this->drcFireHose.Load(this->world, this->sdf);

  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();

  // Harness the Robot
  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  if (this->atlas.isInitialized)
  {
    if (atlas.startupMode == "bdi_stand")
    {
      this->atlas.startupBDIStand = true;
      this->SetRobotMode("bdi_stand");
    }
    else
    {
      this->SetRobotMode("pinned");
      this->atlas.startupHarness = true;
      ROS_INFO("Start robot with gravity turned off and harnessed.");
      if (math::equal(this->atlas.startupHarnessDuration, 0.0))
        ROS_INFO("Atlas will stay pinned.");
      else
        ROS_INFO("Resume to nominal mode after %f seconds.",
          this->atlas.startupHarnessDuration);
    }
  }

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
  }
  else if (_str == "harnessed")
  {
    bool paused = this->world->IsPaused();
    this->world->SetPaused(true);

    // remove pin
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);

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
      atlasPose.pos.z = groundHeight + 1.11;
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
      gzwarn << "No entity below robot, or GetEntityBelowPoint returned NULL pointer.\n";
      // put atlas back
      this->atlas.model->SetLinkWorldPose(atlasPose, this->atlas.pinLink);
    }
    this->world->SetPaused(paused);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
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
  }
  else if (_str == "bdi_stand")
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
  this->atlas.model->SetWorldPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  /// \todo: get these from incoming message
  std::string modelName = "fire_hose";
  std::string linkName = "coupling";
  std::string gripperName = "r_hand";
  math::Pose relPose(math::Vector3(0, -0.3, -0.1),
               math::Quaternion(0, 0, 0));

  physics::ModelPtr grabModel = this->world->GetModel(modelName);
  if (grabModel)
  {
    physics::LinkPtr object = grabModel->GetLink(linkName);
    if (object)
    {
      physics::LinkPtr gripper = this->atlas.model->GetLink(gripperName);
      if (gripper)
      {
        // teleports the object being attached together
        math::Pose pose = relPose + gripper->GetWorldPose();
        grabModel->SetLinkWorldPose(pose, object);

        if (!this->grabJoint)
          this->grabJoint = this->AddJoint(this->world, this->atlas.model,
                                           gripper, object,
                                           "revolute",
                                           math::Vector3(0, 0, 0),
                                           math::Vector3(0, 0, 1),
                                           0.0, 0.0);
      }
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
                                      double _upper, double _lower)
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

/*
  // disable collision between the link pair
  if (_link1)
    _link1->SetCollideMode("fixed");
  if (_link2)
    _link2->SetCollideMode("fixed");
*/
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
  // with PID stand in BDI stand pose.
  // After startupStandPrepDuration - 1 seconds, start StandPrep mode
  // After startupStandPrepDuration seconds, start Stand mode
  if (this->atlas.startupBDIStand && this->atlas.isInitialized)
  {
    if (curTime > atlas.startupStandPrepDuration)
    {
      this->atlasCommandController.SetBDIStand();
      this->atlas.startupBDIStand = false;
    }
    else if (!this->bdiStandPrep && curTime >
      atlas.startupStandPrepDuration - 1)
    {
      this->atlasCommandController.SetBDIStandPrep();
      this->bdiStandPrep = true;
    }
  }

  if (this->atlas.startupHarness && this->atlas.isInitialized &&
      !math::equal(atlas.startupHarnessDuration, 0.0) &&
      curTime > atlas.startupHarnessDuration)
  {
    this->SetRobotMode("nominal");
    this->atlas.startupHarness = false;
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
  std::string fireHoseModelName = sdf->GetValueString("fire_hose_model");
  this->fireHoseModel = _world->GetModel(fireHoseModelName);
  if (!this->fireHoseModel)
  {
    ROS_INFO("fire_hose_model [%s] not found", fireHoseModelName.c_str());
    return;
  }
  this->initialFireHosePose = this->fireHoseModel->GetWorldPose();

  // Get coupling link
  std::string couplingLinkName = sdf->GetValueString("coupling_link");
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
  std::string standpipeModelName = sdf->GetValueString("standpipe_model");
  this->standpipeModel = _world->GetModel(standpipeModelName);
  if (!this->standpipeModel)
  {
    ROS_ERROR("standpipe model [%s] not found", standpipeModelName.c_str());
    return;
  }

  // Get spout link
  std::string spoutLinkName = sdf->GetValueString("spout_link");
  this->spoutLink = this->standpipeModel->GetLink(spoutLinkName);
  if (!this->spoutLink)
  {
    ROS_ERROR("spout link [%s] not found", spoutLinkName.c_str());
    return;
  }

  this->threadPitch = sdf->GetValueDouble("thread_pitch");

  this->couplingRelativePose = sdf->GetValuePose("coupling_relative_pose");

  // Set initial configuration
  this->SetInitialConfiguration();

  this->isInitialized = true;
}

void VRCPlugin::CheckThreadStart()
{
  if (!this->drcFireHose.isInitialized)
    return;

  // gzerr << "coupling [" << this->couplingLink->GetWorldPose() << "]\n";
  // gzerr << "spout [" << this->spoutLink->GetWorldPose() << "]\n"
  math::Pose connectPose(this->drcFireHose.couplingRelativePose);
  math::Pose relativePose = this->drcFireHose.couplingLink->GetWorldPose() -
                            this->drcFireHose.spoutLink->GetWorldPose();

  math::Pose connectOffset = relativePose - connectPose;

  double posErr = (relativePose.pos - connectPose.pos).GetLength();
  double rotErr = (relativePose.rot.GetZAxis() -
                   connectPose.rot.GetZAxis()).GetLength();

  // gzdbg << "connect offset [" << connectOffset
  //       << "] xyz [" << posErr
  //       << "] rpy [" << rotErr
  //       << "]\n";

  if (!this->drcFireHose.screwJoint)
  {
    if (posErr < 0.01 && rotErr < 0.01)
    {
      this->drcFireHose.screwJoint =
        this->AddJoint(this->world, this->drcFireHose.fireHoseModel,
                       this->drcFireHose.spoutLink,
                       this->drcFireHose.couplingLink,
                       "screw",
                       math::Vector3(0, 0, 0),
                       math::Vector3(0, 0, 1),
                       20.0/1000, -0.5/1000);
                       // 20.0, -0.5); // recover threadPitch
    }
  }
  else
  {
    // check joint position to disconnect
    double position = this->drcFireHose.screwJoint->GetAngle(0).Radian();
    // gzerr << "position " << position << "\n";
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
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <drc_vehicle><model_name> blocks. using default.");
    this->model = _world->GetModel("drc_vehicle");
  }

  if (!this->model)
  {
    ROS_INFO("drc vehicle not found.");
    return;
  }

  if (_sdf->HasElement("drc_vehicle") &&
      _sdf->GetElement("drc_vehicle")->HasElement("seat_link"))
  {
    this->seatLink = this->model->GetLink(_sdf->GetElement("drc_vehicle")
                        ->GetValueString("seat_link"));
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
void VRCPlugin::Robot::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;
  this->startupHarnessDuration = 10;
  this->startupStandPrepDuration = 2;
  this->startupHarness = false;
  this->startupBDIStand = false;
  this->startupMode = "bdi_stand";

  // load parameters
  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("atlas")
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><model_name> blocks. using default.");
    this->model = _world->GetModel("atlas");
  }

  if (!this->model)
  {
    ROS_ERROR("atlas model not found.");
    return;
  }

  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("pin_link"))
  {
    this->pinLink = this->model->GetLink(_sdf->GetElement("atlas")
                        ->GetValueString("pin_link"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><pin_link> blocks, using default.");
    this->pinLink = this->model->GetLink("utorso");
  }

  if (!this->pinLink)
  {
    ROS_ERROR("atlas robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadVRCROSAPI()
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
VRCPlugin::AtlasCommandController::AtlasCommandController()
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

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // must match those inside AtlasPlugin
  this->jointNames.push_back("atlas::back_lbz");
  this->jointNames.push_back("atlas::back_mby");
  this->jointNames.push_back("atlas::back_ubx");
  this->jointNames.push_back("atlas::neck_ay");
  this->jointNames.push_back("atlas::l_leg_uhz");
  this->jointNames.push_back("atlas::l_leg_mhx");
  this->jointNames.push_back("atlas::l_leg_lhy");
  this->jointNames.push_back("atlas::l_leg_kny");
  this->jointNames.push_back("atlas::l_leg_uay");
  this->jointNames.push_back("atlas::l_leg_lax");
  this->jointNames.push_back("atlas::r_leg_uhz");
  this->jointNames.push_back("atlas::r_leg_mhx");
  this->jointNames.push_back("atlas::r_leg_lhy");
  this->jointNames.push_back("atlas::r_leg_kny");
  this->jointNames.push_back("atlas::r_leg_uay");
  this->jointNames.push_back("atlas::r_leg_lax");
  this->jointNames.push_back("atlas::l_arm_usy");
  this->jointNames.push_back("atlas::l_arm_shx");
  this->jointNames.push_back("atlas::l_arm_ely");
  this->jointNames.push_back("atlas::l_arm_elx");
  this->jointNames.push_back("atlas::l_arm_uwy");
  this->jointNames.push_back("atlas::l_arm_mwx");
  this->jointNames.push_back("atlas::r_arm_usy");
  this->jointNames.push_back("atlas::r_arm_shx");
  this->jointNames.push_back("atlas::r_arm_ely");
  this->jointNames.push_back("atlas::r_arm_elx");
  this->jointNames.push_back("atlas::r_arm_uwy");
  this->jointNames.push_back("atlas::r_arm_mwx");

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

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, this->jointNames[i], boost::is_any_of(":"));

    double val;
    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/p", val);
    this->ac.kp_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/i", val);
    this->ac.ki_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/d", val);
    this->ac.kd_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
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

  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1,
    boost::bind(&AtlasCommandController::GetJointStates, this, _1),
    ros::VoidPtr(), this->rosNode->getCallbackQueue());
  this->subJointStates =
    this->rosNode->subscribe(jointStatesSo);
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
    jps.insert(std::make_pair(this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIStandPrep()
{
  atlas_msgs::AtlasSimInterfaceCommand ac;
  ac.header.stamp = ros::Time::now();
  ac.behavior = ac.STAND_PREP;
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
    jps.insert(std::make_pair(this->jointNames[i], this->ac.position[i]));

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
    jps.insert(std::make_pair(this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}
}
