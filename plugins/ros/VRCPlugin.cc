/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2012 Open Source Robotics Foundation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Plugin to allow development shortcuts for VRC competition.
 * Author: John Hsu and Steven Peters
 * Date: December 2012
 */

#include "VRCPlugin.hh"

namespace gazebo
{

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
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->ros_queue_.clear();
  this->ros_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world = _parent;
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&VRCPlugin::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::LoadThread()
{

  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("");

  // load VRC ROS API
  this->LoadVRCROSAPI();

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->robotCmdVel = geometry_msgs::Twist();

  // Load Robot
  this->drc_robot.Load(this->world, this->sdf);

  // Load Vehicle
  this->drc_vehicle.Load(this->world, this->sdf);

  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();

  // Harness the Robot
  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  this->SetRobotMode("pinned");
  this->drc_robot.startupHarness = true;
  ROS_INFO("Start robot with gravity turned off and harnessed.");
  ROS_INFO("Resume to nominal mode after 10 seconds.");



  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &VRCPlugin::ROSQueueThread,this ) );

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
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
    physics::Link_V links = this->drc_robot.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    if (this->drc_robot.pinJoint)
      this->RemoveJoint(this->drc_robot.pinJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->drc_robot.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    if (this->drc_robot.pinJoint)
      this->RemoveJoint(this->drc_robot.pinJoint);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
    if (!this->drc_robot.pinJoint)
      this->drc_robot.pinJoint = this->AddJoint(this->world,
                                        this->drc_robot.model,
                                        physics::LinkPtr(),
                                        this->drc_robot.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->drc_robot.initialPose = this->drc_robot.pinLink->GetWorldPose();

    physics::Link_V links = this->drc_robot.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
  }
  else if (_str == "nominal")
  {
    // reinitialize pinning
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->drc_robot.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->drc_robot.pinJoint)
      this->RemoveJoint(this->drc_robot.pinJoint);
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
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z),
                  math::Quaternion(_pose->orientation.w,
                                   _pose->orientation.x,
                                   _pose->orientation.y,
                                   _pose->orientation.z));
  this->drc_robot.model->SetWorldPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotGrabLink(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
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
      physics::LinkPtr gripper = this->drc_robot.model->GetLink(gripperName);
      if (gripper)
      {
        // teleports the object being attached together
        math::Pose pose = relPose + gripper->GetWorldPose();
        grabModel->SetLinkWorldPose(pose, object);

        if (!this->grabJoint)
          this->grabJoint = this->AddJoint(this->world, this->drc_robot.model,
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
void VRCPlugin::RobotEnterCar(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  if (this->drc_robot.pinJoint)
    this->RemoveJoint(this->drc_robot.pinJoint);

  this->drc_robot.vehicleRelPose = math::Pose(math::Vector3(0.52, 0.5, 2),
                                              math::Quaternion());

  this->drc_robot.model->SetLinkWorldPose(
    this->drc_robot.vehicleRelPose + this->drc_vehicle.model->GetWorldPose(),
    this->drc_robot.pinLink);

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drc_vehicle.model,
                                       this->drc_vehicle.seatLink,
                                       this->drc_robot.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
/*
  std::map<std::string, double> jointPositions;
  jointPositions["drc_robot::back_lbz" ] =  0.00;
  jointPositions["drc_robot::back_mby" ] =  0.00;
  jointPositions["drc_robot::back_ubx" ] =  0.00;
  jointPositions["drc_robot::neck_ay"  ] =  0.00;
  jointPositions["drc_robot::l_leg_uhz"] =  0.00;
  jointPositions["drc_robot::l_leg_mhx"] =  0.00;
  jointPositions["drc_robot::l_leg_lhy"] = -1.80;
  jointPositions["drc_robot::l_leg_kny"] =  1.80;
  jointPositions["drc_robot::l_leg_uay"] =  0.00;
  jointPositions["drc_robot::l_leg_lax"] =  0.00;
  jointPositions["drc_robot::r_leg_uhz"] =  0.00;
  jointPositions["drc_robot::r_leg_mhx"] =  0.00;
  jointPositions["drc_robot::r_leg_lhy"] = -1.80;
  jointPositions["drc_robot::r_leg_kny"] =  1.80;
  jointPositions["drc_robot::r_leg_uay"] =  0.00;
  jointPositions["drc_robot::r_leg_lax"] =  0.00;
  jointPositions["drc_robot::l_arm_elx"] =  0.00;
  jointPositions["drc_robot::l_arm_ely"] =  0.00;
  jointPositions["drc_robot::l_arm_mwx"] =  0.00;
  jointPositions["drc_robot::l_arm_shx"] =  0.00;
  jointPositions["drc_robot::l_arm_usy"] = -1.60;
  jointPositions["drc_robot::l_arm_uwy"] =  0.00;
  jointPositions["drc_robot::r_arm_elx"] =  0.00;
  jointPositions["drc_robot::r_arm_ely"] =  0.00;
  jointPositions["drc_robot::r_arm_mwx"] =  0.00;
  jointPositions["drc_robot::r_arm_shx"] =  0.00;
  jointPositions["drc_robot::r_arm_usy"] =  1.60;
  jointPositions["drc_robot::r_arm_uwy"] =  0.00;
  this->drc_robot.model->SetJointPositions(jointPositions);
*/

  // wait for action server to come up
  while(!this->joint_trajectory_controller.traj_client_->waitForServer(
    ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  this->joint_trajectory_controller.startTrajectory(
    this->joint_trajectory_controller.seatingConfiguration());

  // Wait for trajectory completion
  while(!joint_trajectory_controller.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    usleep(50000);
  }
  ROS_INFO("set configuration done");

  this->drc_robot.vehicleRelPose = math::Pose(math::Vector3(0.52, 0.5, 1.27),
                                              math::Quaternion());

  this->RemoveJoint(this->vehicleRobotJoint);

  this->drc_robot.model->SetLinkWorldPose(
    this->drc_robot.vehicleRelPose + this->drc_vehicle.model->GetWorldPose(),
    this->drc_robot.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drc_vehicle.model,
                                       this->drc_vehicle.seatLink,
                                       this->drc_robot.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotExitCar(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  if (this->drc_robot.pinJoint)
    this->RemoveJoint(this->drc_robot.pinJoint);

  this->drc_robot.vehicleRelPose = math::Pose(math::Vector3(0.52, 1.7, 1.20),
                                              math::Quaternion());

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  this->drc_robot.model->SetLinkWorldPose(
    this->drc_robot.vehicleRelPose + this->drc_vehicle.model->GetWorldPose(),
    this->drc_robot.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drc_vehicle.model,
                                       this->drc_vehicle.seatLink,
                                       this->drc_robot.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);

  // wait for action server to come up
  while(!this->joint_trajectory_controller.traj_client_->waitForServer(
    ros::Duration(1.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  this->joint_trajectory_controller.startTrajectory(
    this->joint_trajectory_controller.standingConfiguration());

  // Wait for trajectory completion
  while(!joint_trajectory_controller.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    usleep(50000);
  }
  ROS_INFO("set configuration done");

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);
}


////////////////////////////////////////////////////////////////////////////////
// remove a joint
void VRCPlugin::RemoveJoint(physics::JointPtr &_joint)
{
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
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose,
                       const std::map<std::string, double> &/*_jointPositions*/)
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
                               this->drc_robot.pinLink,
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
  double cur_time = this->world->GetSimTime().Double();

  if (this->drc_robot.startupHarness && cur_time > 10)
  {
    this->SetRobotMode("nominal");
    this->drc_robot.startupHarness = false;
  }

  if (this->warpRobotWithCmdVel && cur_time - this->lastUpdateTime >= 0)
  {
    double dt = cur_time - this->lastUpdateTime;
    if (dt > 0)
    {
      this->lastUpdateTime = cur_time;
      math::Pose cur_pose = this->drc_robot.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->drc_robot.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      this->Teleport(this->drc_robot.pinLink,
                     this->drc_robot.pinJoint,
                     new_pose);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::ROSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Vehicle::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
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
    ROS_ERROR("drc vehicle not found.");
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
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Robot::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // load parameters
  if (_sdf->HasElement("drc_robot") &&
      _sdf->GetElement("drc_robot")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("drc_robot")
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <drc_robot><model_name> blocks. using default.");
    this->model = _world->GetModel("drc_robot");
  }

  if (!this->model)
  {
    ROS_ERROR("drc robot not found.");
    return;
  }

  if (_sdf->HasElement("drc_robot") &&
      _sdf->GetElement("drc_robot")->HasElement("pin_link"))
  {
    this->pinLink = this->model->GetLink(_sdf->GetElement("drc_robot")
                        ->GetValueString("pin_link"));
  }
  else
  {
    ROS_INFO("Can't find <drc_robot><pin_link> blocks, using default.");
    this->pinLink = this->model->GetLink("utorso");
  }

  if (!this->pinLink)
  {
    ROS_ERROR("drc robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadVRCROSAPI()
{
  // ros subscription
  std::string robot_enter_car_topic_name = "drc_world/robot_enter_car";
  ros::SubscribeOptions robot_enter_car_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_enter_car_topic_name, 100,
    boost::bind( &VRCPlugin::RobotEnterCar,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->robot_enter_car_sub_ = this->rosnode_->subscribe(robot_enter_car_so);

  std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
  ros::SubscribeOptions robot_exit_car_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_exit_car_topic_name, 100,
    boost::bind( &VRCPlugin::RobotExitCar,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->robot_exit_car_sub_ = this->rosnode_->subscribe(robot_exit_car_so);

  std::string robot_grab_topic_name = "drc_world/robot_grab_link";
  ros::SubscribeOptions robot_grab_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_grab_topic_name, 100,
    boost::bind( &VRCPlugin::RobotGrabLink,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->robot_grab_sub_ = this->rosnode_->subscribe(robot_grab_so);

  std::string robot_release_topic_name = "drc_world/robot_release_link";
  ros::SubscribeOptions robot_release_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_release_topic_name, 100,
    boost::bind( &VRCPlugin::RobotReleaseLink,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->robot_release_sub_ = this->rosnode_->subscribe(robot_release_so);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadRobotROSAPI()
{
  // ros subscription
  std::string trajectory_topic_name = "drc_robot/cmd_vel";
  ros::SubscribeOptions trajectory_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100,
    boost::bind( &VRCPlugin::SetRobotCmdVel,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->drc_robot.trajectory_sub_ = this->rosnode_->subscribe(trajectory_so);

  std::string pose_topic_name = "drc_robot/pose";
  ros::SubscribeOptions pose_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    pose_topic_name, 100,
    boost::bind( &VRCPlugin::SetRobotPose,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->drc_robot.pose_sub_ = this->rosnode_->subscribe(pose_so);

  std::string configuration_topic_name = "drc_robot/configuration";
  ros::SubscribeOptions configuration_so =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    configuration_topic_name, 100,
    boost::bind( &VRCPlugin::SetRobotConfiguration,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->drc_robot.configuration_sub_ =
    this->rosnode_->subscribe(configuration_so);

  std::string mode_topic_name = "drc_robot/mode";
  ros::SubscribeOptions mode_so =
    ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100,
    boost::bind( &VRCPlugin::SetRobotModeTopic,this,_1),
    ros::VoidPtr(), &this->ros_queue_);
  this->drc_robot.mode_sub_ = this->rosnode_->subscribe(mode_so);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
  &/* _cmd */)
{
  // This function is planned but not yet implemented.
  ROS_ERROR("The /drc_robot/configuration handler is not implemented.\n");
}

GZ_REGISTER_WORLD_PLUGIN(VRCPlugin)
}
