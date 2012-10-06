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
  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;
  this->world_->EnablePhysicsEngine(true);

  // this->world_->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->last_update_time_ = this->world_->GetSimTime().Double();
  this->last_cmd_vel_update_time_ = this->world_->GetSimTime().Double();
  this->cmd_vel_ = geometry_msgs::Twist();

  this->FixLink(this->model_->GetLink("pelvis"));

  // ros subscription
  std::string topic_name = "/cmd_vel";
  ros::SubscribeOptions trajectory_so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    topic_name, 100, boost::bind( &DRCRobotPlugin::SetRobotCmdVel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->ros_sub_ = this->rosnode_->subscribe(trajectory_so);

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread( boost::bind( &DRCRobotPlugin::QueueThread,this ) );

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
     boost::bind(&DRCRobotPlugin::UpdateStates, this));
}

void DRCRobotPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  this->cmd_vel_ = *_cmd;
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void DRCRobotPlugin::FixLink(physics::LinkPtr link)
{
/*
  std::ostringstream newJointStr;
  newJointStr
    << "<joint name='fix_a_link_jont' type='revolute'>"
    << "  <parent>world</parent>"
    << "  <child>" << link->GetName() << "</child>"
    << "  <axis>"
    << "    <xyz>0 0 1</xyz>"
    << "    <limits>"
    << "      <lower>0</lower>"
    << "      <upper>0</upper>"
    << "    </limits>"
    << "  </axis>"
    << "</joint>";

*/


  this->joint_ = this->world_->GetPhysicsEngine()->CreateJoint("revolute",this->model_);
  this->joint_->Attach(physics::LinkPtr(), link);

  math::Pose  anchor_pose(math::Vector3(0, 0, 0.2),
                          math::Quaternion(1, 0, 0, 0));
  this->joint_->Load(physics::LinkPtr(), link, anchor_pose);
  this->joint_->SetAxis(0, math::Vector3(0, 0, 1));
  this->joint_->SetHighStop(0, 0);
  this->joint_->SetLowStop(0, 0);
  this->joint_->SetAnchor(0, anchor_pose.pos);
  this->joint_->Init();
}

////////////////////////////////////////////////////////////////////////////////
// unglue a link to the world by destroying the fixed joint
void DRCRobotPlugin::UnfixLink()
{
  this->joint_.reset();
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

  if (cur_time - this->last_cmd_vel_update_time_ >= 0.01)
  {
    double dt = cur_time - this->last_cmd_vel_update_time_;
    if (dt > 0)
    {
      this->last_cmd_vel_update_time_ = cur_time;
      math::Pose cur_pose = this->model_->GetWorldPose();
      math::Pose new_pose;
      new_pose.pos.x = cur_pose.pos.x + this->cmd_vel_.linear.x * dt;
      new_pose.pos.y = cur_pose.pos.y + this->cmd_vel_.linear.y * dt;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      rpy.z = rpy.z + this->cmd_vel_.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);
    }
  }

  if (false && cur_time - this->last_update_time_ >= 0.01) 
  {
    this->last_update_time_ = cur_time;
    std::map<std::string, double> joint_position_map;

/*
    joint_position_map["drc_robot::back.lbz"] = 0.0;
    joint_position_map["drc_robot::back.mby"] = 0.0;
    joint_position_map["drc_robot::back.ubx"] = 0.0;
    joint_position_map["drc_robot::neck.ay"] = 0.0;
*/

    joint_position_map["drc_robot::l.leg.uhz"] = 0.0;
    joint_position_map["drc_robot::l.leg.mhx"] = 0.0;
    joint_position_map["drc_robot::l.leg.lhy"] = 0.0;
    joint_position_map["drc_robot::l.leg.kny"] = 0.0;
    joint_position_map["drc_robot::l.leg.uay"] = 0.0;
    joint_position_map["drc_robot::l.leg.lax"] = 0.0;

    joint_position_map["drc_robot::r.leg.lax"] = 0.0;
    joint_position_map["drc_robot::r.leg.uay"] = 0.0;
    joint_position_map["drc_robot::r.leg.kny"] = 0.0;
    joint_position_map["drc_robot::r.leg.lhy"] = 0.0;
    joint_position_map["drc_robot::r.leg.mhx"] = 0.0;
    joint_position_map["drc_robot::r.leg.uhz"] = 0.0;

  /*
    joint_position_map["drc_robot::l.arm.elx"] = 0.0;
    joint_position_map["drc_robot::l.arm.ely"] = 0.0;
    joint_position_map["drc_robot::l.arm.mwx"] = 0.0;
    joint_position_map["drc_robot::l.arm.shx"] = 0.0;
    joint_position_map["drc_robot::l.arm.usy"] = 0.0;
    joint_position_map["drc_robot::l.arm.uwy"] = 0.0;
    joint_position_map["drc_robot::r.arm.elx"] = 0.0;
    joint_position_map["drc_robot::r.arm.ely"] = 0.0;
    joint_position_map["drc_robot::r.arm.mwx"] = 0.0;
    joint_position_map["drc_robot::r.arm.shx"] = 0.0;
    joint_position_map["drc_robot::r.arm.usy"] = 0.0;
    joint_position_map["drc_robot::r.arm.uwy"] = 0.0;
  */

  /*
    joint_position_map["drc_robot::r_camhand_joint"] = 0.0;
    joint_position_map["drc_robot::r_f0_base"] = 0.0;
    joint_position_map["drc_robot::r_f0_j0"] = 0.0;
    joint_position_map["drc_robot::r_f0_j1"] = 0.0;
    joint_position_map["drc_robot::r_f0_j2"] = 0.0;
    joint_position_map["drc_robot::r_f0_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::r_f0_1_accel"] = 0.0;
    joint_position_map["drc_robot::r_f0_2_accel"] = 0.0;
    joint_position_map["drc_robot::r_f1_base"] = 0.0;
    joint_position_map["drc_robot::r_f1_j0"] = 0.0;
    joint_position_map["drc_robot::r_f1_j1"] = 0.0;
    joint_position_map["drc_robot::r_f1_j2"] = 0.0;
    joint_position_map["drc_robot::r_f1_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::r_f1_1_accel"] = 0.0;
    joint_position_map["drc_robot::r_f1_2_accel"] = 0.0;
    joint_position_map["drc_robot::r_f2_base"] = 0.0;
    joint_position_map["drc_robot::r_f2_j0"] = 0.0;
    joint_position_map["drc_robot::r_f2_j1"] = 0.0;
    joint_position_map["drc_robot::r_f2_j2"] = 0.0;
    joint_position_map["drc_robot::r_f2_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::r_f2_1_accel"] = 0.0;
    joint_position_map["drc_robot::r_f2_2_accel"] = 0.0;
    joint_position_map["drc_robot::r_f3_base"] = 0.0;
    joint_position_map["drc_robot::r_f3_j0"] = 0.0;
    joint_position_map["drc_robot::r_f3_j1"] = 0.0;
    joint_position_map["drc_robot::r_f3_j2"] = 0.0;
    joint_position_map["drc_robot::r_f3_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::r_f3_1_accel"] = 0.0;
    joint_position_map["drc_robot::r_f3_2_accel"] = 0.0;

    joint_position_map["drc_robot::l_camhand_joint"] = 0.0;
    joint_position_map["drc_robot::l_f0_base"] = 0.0;
    joint_position_map["drc_robot::l_f0_j0"] = 0.0;
    joint_position_map["drc_robot::l_f0_j1"] = 0.0;
    joint_position_map["drc_robot::l_f0_j2"] = 0.0;
    joint_position_map["drc_robot::l_f0_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::l_f0_1_accel"] = 0.0;
    joint_position_map["drc_robot::l_f0_2_accel"] = 0.0;
    joint_position_map["drc_robot::l_f1_base"] = 0.0;
    joint_position_map["drc_robot::l_f1_j0"] = 0.0;
    joint_position_map["drc_robot::l_f1_j1"] = 0.0;
    joint_position_map["drc_robot::l_f1_j2"] = 0.0;
    joint_position_map["drc_robot::l_f1_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::l_f1_1_accel"] = 0.0;
    joint_position_map["drc_robot::l_f1_2_accel"] = 0.0;
    joint_position_map["drc_robot::l_f2_base"] = 0.0;
    joint_position_map["drc_robot::l_f2_j0"] = 0.0;
    joint_position_map["drc_robot::l_f2_j1"] = 0.0;
    joint_position_map["drc_robot::l_f2_j2"] = 0.0;
    joint_position_map["drc_robot::l_f2_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::l_f2_1_accel"] = 0.0;
    joint_position_map["drc_robot::l_f2_2_accel"] = 0.0;
    joint_position_map["drc_robot::l_f3_base"] = 0.0;
    joint_position_map["drc_robot::l_f3_j0"] = 0.0;
    joint_position_map["drc_robot::l_f3_j1"] = 0.0;
    joint_position_map["drc_robot::l_f3_j2"] = 0.0;
    joint_position_map["drc_robot::l_f3_fixed_accel"] = 0.0;
    joint_position_map["drc_robot::l_f3_1_accel"] = 0.0;
    joint_position_map["drc_robot::l_f3_2_accel"] = 0.0;
  */

    // this->model_->SetJointPositions(joint_position_map);

    // math::Pose pose(2, 1, 1.5, 0, 0, 0);
    // this->model_->SetLinkWorldPose(pose, "pelvis");

    this->world_->EnablePhysicsEngine(false);

    this->model_->GetJoint("drc_robot::l.leg.uhz")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::l.leg.mhx")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::l.leg.lhy")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::l.leg.kny")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::l.leg.uay")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::l.leg.lax")->SetAngle(0, 0.0);

    this->model_->GetJoint("drc_robot::r.leg.lax")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::r.leg.uay")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::r.leg.kny")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::r.leg.lhy")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::r.leg.mhx")->SetAngle(0, 0.0);
    this->model_->GetJoint("drc_robot::r.leg.uhz")->SetAngle(0, 0.0);
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
