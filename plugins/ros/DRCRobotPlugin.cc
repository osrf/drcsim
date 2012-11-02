/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include "DRCRobotPlugin.hh"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCRobotPlugin::DRCRobotPlugin()
{
  /// initial anchor pose
  this->anchorPose = math::Vector3(0, 0, 0);
  this->warpRobot = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCRobotPlugin::~DRCRobotPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCRobotPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // initialize ros
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",
      ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;
  this->world->EnablePhysicsEngine(true);

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->cmdVel = geometry_msgs::Twist();

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->fixedLink = this->model->GetLink("pelvis");
  this->initialPose = this->fixedLink->GetWorldPose();
  this->FixLink(this->fixedLink);

  // ros subscription
  std::string trajectory_topic_name = "/cmd_vel";
  ros::SubscribeOptions trajectory_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100,
    boost::bind( &DRCRobotPlugin::SetRobotCmdVel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->trajectory_sub_ = this->rosnode_->subscribe(trajectory_so);

  std::string mode_topic_name = "/mode";
  ros::SubscribeOptions mode_so =
    ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100, boost::bind( &DRCRobotPlugin::SetPluginMode,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->mode_sub_ = this->rosnode_->subscribe(mode_so);

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &DRCRobotPlugin::QueueThread,this ) );

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&DRCRobotPlugin::UpdateStates, this));
}

void DRCRobotPlugin::SetPluginMode(const std_msgs::String::ConstPtr &_str)
{
  if (_str->data == "gravity")
  {
    // stop warping robot
    this->warpRobot = false;
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    this->UnfixLink();
  }
  else if (_str->data == "feet")
  {
    // stop warping robot
    this->warpRobot = false;
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    this->UnfixLink();
  }
  else if (_str->data == "pinned")
  {
    // reinitialize pinning
    this->initialPose = this->fixedLink->GetWorldPose();

    this->FixLink(this->fixedLink);

    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
  }
  else if (_str->data == "unpinned")
  {
    // reinitialize pinning
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    this->UnfixLink();
  }
  else
  {
    ROS_INFO("available modes:gravity, feet, pinned, unpinned");
  }

}

void DRCRobotPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobot = false;
  }
  else
  {
    this->cmdVel = *_cmd;
    this->warpRobot = true;
    this->lastUpdateTime = this->world->GetSimTime().Double();
  }
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void DRCRobotPlugin::FixLink(physics::LinkPtr link)
{
  if (!this->fixedJoint)
  {
    this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "revolute",this->model);
    this->fixedJoint->Attach(physics::LinkPtr(), link);
    // load adds the joint to a vector of shared pointers kept
    // in parent and child links, preventing this from being destroyed.
    this->fixedJoint->Load(physics::LinkPtr(), link,
      math::Pose(this->anchorPose, math::Quaternion()));
    this->fixedJoint->SetAxis(0, math::Vector3(0, 0, 1));
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);
    this->fixedJoint->SetAnchor(0, this->anchorPose);
    this->fixedJoint->SetName(link->GetName()+std::string("_world_joint"));
    this->fixedJoint->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// unglue a link to the world by destroying the fixed joint
void DRCRobotPlugin::UnfixLink()
{
  if (this->fixedJoint)
  {
    this->fixedJoint->Detach();
    this->fixedJoint.reset();
  }
}

void DRCRobotPlugin::WarpDRCRobot(math::Pose _pose)
{
  // two ways, both requres to be done in a single step, is pause reliable? or do we
  // need some mutex.
  //   1. update poses, set the joint anchor offset properties,
  //      this requires introducing a new SetAnchor call with the
  //      new joint agnle
  //      will not do for now.
  //      using: this->fixedJoint->SetAnchor(0, _pose);
  //   or,
  //   2. less ideally, pause, break joint, update pose,
  //       create new joint, unpause

  // try 2. here
  bool p = this->world->IsPaused();
  bool e = this->world->GetEnablePhysicsEngine();
  this->world->EnablePhysicsEngine(false);
  this->world->SetPaused(true);
  this->UnfixLink();
  this->model->SetLinkWorldPose(_pose, this->fixedLink);
  this->FixLink(this->fixedLink);
  this->world->SetPaused(p);
  this->world->EnablePhysicsEngine(e);
}

// Set DRC Robot feet placement
void DRCRobotPlugin::SetFeetPose(math::Pose _lPose, math::Pose _rPose)
{
  physics::LinkPtr l_foot = this->model->GetLink("l_foot");
  physics::LinkPtr r_foot = this->model->GetLink("r_foot");
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCRobotPlugin::UpdateStates()
{
  double cur_time = this->world->GetSimTime().Double();

  if (this->warpRobot && cur_time - this->lastUpdateTime >= 0)
  {
    double dt = cur_time - this->lastUpdateTime;
    if (dt > 0)
    {
      this->lastUpdateTime = cur_time;
      math::Pose cur_pose = this->fixedLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->cmdVel.linear.x, this->cmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->cmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the fixed joint
      this->WarpDRCRobot(new_pose);
    }
  }
}

void DRCRobotPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCRobotPlugin)
}
