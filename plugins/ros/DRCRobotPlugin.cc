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
  this->anchor_pose_ = math::Vector3(0, 0, 0);
  this->warp_robot_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCRobotPlugin::~DRCRobotPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
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
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;
  this->world_->EnablePhysicsEngine(true);

  // this->world_->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->last_cmd_vel_update_time_ = this->world_->GetSimTime().Double();
  this->cmd_vel_ = geometry_msgs::Twist();

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->fixed_link_ = this->model_->GetLink("pelvis");
  this->initial_pose_ = this->fixed_link_->GetWorldPose();
  this->FixLink(this->fixed_link_);

  // ros subscription
  std::string trajectory_topic_name = "/cmd_vel";
  ros::SubscribeOptions trajectory_so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100, boost::bind( &DRCRobotPlugin::SetRobotCmdVel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->trajectory_sub_ = this->rosnode_->subscribe(trajectory_so);

  std::string mode_topic_name = "/mode";
  ros::SubscribeOptions mode_so = ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100, boost::bind( &DRCRobotPlugin::SetPluginMode,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->mode_sub_ = this->rosnode_->subscribe(mode_so);

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread( boost::bind( &DRCRobotPlugin::QueueThread,this ) );

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
     boost::bind(&DRCRobotPlugin::UpdateStates, this));
}

void DRCRobotPlugin::SetPluginMode(const std_msgs::String::ConstPtr &_str)
{

}

void DRCRobotPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warp_robot_ = false;
  }
  else
  {
    this->cmd_vel_ = *_cmd;
    this->warp_robot_ = true;
    this->last_cmd_vel_update_time_ = this->world_->GetSimTime().Double();
  }
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void DRCRobotPlugin::FixLink(physics::LinkPtr link)
{
  this->fixed_joint_ = this->world_->GetPhysicsEngine()->CreateJoint("revolute",this->model_);
  this->fixed_joint_->Attach(physics::LinkPtr(), link);
  // load adds the joint to a vector of shared pointers kept in parent and child links
  // preventing this from being destroyed.  calling load is not necessary anyways.
  this->fixed_joint_->Load(physics::LinkPtr(), link,
    math::Pose(this->anchor_pose_, math::Quaternion()));
  this->fixed_joint_->SetAxis(0, math::Vector3(0, 0, 1));
  this->fixed_joint_->SetHighStop(0, 0);
  this->fixed_joint_->SetLowStop(0, 0);
  this->fixed_joint_->SetAnchor(0, this->anchor_pose_);
  this->fixed_joint_->SetName(link->GetName()+std::string("_world_fixed_joint"));
  this->fixed_joint_->Init();
}

////////////////////////////////////////////////////////////////////////////////
// unglue a link to the world by destroying the fixed joint
void DRCRobotPlugin::UnfixLink()
{
  this->fixed_joint_->Detach();
  this->fixed_joint_.reset();
}

void DRCRobotPlugin::WarpDRCRobot(math::Pose _pose)
{
  // two ways, both requres to be done in a single step, is pause reliable? or do we
  // need some mutex.
  //   1. update poses, set the joint anchor offset properties,
  //      this requires introducing a new SetAnchor call with the new joint agnle
  //      will not do for now.
  //      using: this->fixed_joint_->SetAnchor(0, _pose);
  //   or,
  //   2. less ideally, pause, break joint, update pose, create new joint, unpause

  // try 2. here
  bool p = this->world_->IsPaused();
  bool e = this->world_->GetEnablePhysicsEngine();
  this->world_->EnablePhysicsEngine(false);
  this->world_->SetPaused(true);
  this->UnfixLink();
  this->model_->SetLinkWorldPose(_pose, this->fixed_link_);
  this->FixLink(this->fixed_link_);
  this->world_->SetPaused(p);
  this->world_->EnablePhysicsEngine(e);
}

// Set DRC Robot feet placement
void DRCRobotPlugin::SetFeetPose(math::Pose _l_pose, math::Pose _r_pose)
{
  physics::LinkPtr l_foot = this->model_->GetLink("l_foot");
  physics::LinkPtr r_foot = this->model_->GetLink("r_foot");
}


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCRobotPlugin::UpdateStates()
{
  double cur_time = this->world_->GetSimTime().Double();

  if (this->warp_robot_ && cur_time - this->last_cmd_vel_update_time_ >= 0)
  {
    double dt = cur_time - this->last_cmd_vel_update_time_;
    if (dt == 0)
    {
      gzerr << "dt is 0\n";
    }
    else
    {
      this->last_cmd_vel_update_time_ = cur_time;
      math::Pose cur_pose = this->fixed_link_->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->cmd_vel_.linear.x, this->cmd_vel_.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->initial_pose_.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->cmd_vel_.angular.z * dt;

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
