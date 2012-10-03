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
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCRobotPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;
  this->world_->EnablePhysicsEngine(true);

  // this->world_->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->last_update_time_ = this->world_->GetSimTime().Double();

  this->FixLink(this->model_->GetLink("pelvis"));
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  // this->update_connection_ = event::Events::ConnectWorldUpdateEnd(
  //     boost::bind(&DRCRobotPlugin::UpdateStates, this));
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


  this->joint_ = this->world_->GetPhysicsEngine()->CreateJoint("revolute");
  this->joint_->SetModel(this->model_);
  this->joint_->Attach(physics::LinkPtr(), link);
  math::Pose pose = link->GetWorldPose();
  // math::Pose  pose(math::Vector3(0, 0, 0.2), math::Quaternion(1, 0, 0, 0));
  this->joint_->Load(physics::LinkPtr(), link, pose);
  this->joint_->SetAxis(0, math::Vector3(0, 0, 1));
  this->joint_->SetHighStop(0, 0);
  this->joint_->SetLowStop(0, 0);
  this->joint_->SetAnchor(0, pose.pos);
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

  //if (cur_time - this->last_update_time_ >= 0.01) 
  {
    this->last_update_time_ = cur_time;
    std::map<std::string, double> joint_position_map;

/*
    joint_position_map["back.lbz"] = 0.0;
    joint_position_map["back.mby"] = 0.0;
    joint_position_map["back.ubx"] = 0.0;
    joint_position_map["neck.ay"] = 0.0;
*/

    joint_position_map["l.leg.kny"] = 0.0;
    joint_position_map["l.leg.lax"] = 0.0;
    joint_position_map["l.leg.lhy"] = 0.0;
    joint_position_map["l.leg.mhx"] = 0.0;
    joint_position_map["l.leg.uay"] = 0.0;
    joint_position_map["l.leg.uhz"] = 0.0;

    joint_position_map["r.leg.kny"] = 0.0;
    joint_position_map["r.leg.lax"] = 0.0;
    joint_position_map["r.leg.lhy"] = 0.0;
    joint_position_map["r.leg.mhx"] = 0.0;
    joint_position_map["r.leg.uay"] = 0.0;
    joint_position_map["r.leg.uhz"] = 0.0;

  /*
    joint_position_map["l.arm.elx"] = 0.0;
    joint_position_map["l.arm.ely"] = 0.0;
    joint_position_map["l.arm.mwx"] = 0.0;
    joint_position_map["l.arm.shx"] = 0.0;
    joint_position_map["l.arm.usy"] = 0.0;
    joint_position_map["l.arm.uwy"] = 0.0;
    joint_position_map["r.arm.elx"] = 0.0;
    joint_position_map["r.arm.ely"] = 0.0;
    joint_position_map["r.arm.mwx"] = 0.0;
    joint_position_map["r.arm.shx"] = 0.0;
    joint_position_map["r.arm.usy"] = 0.0;
    joint_position_map["r.arm.uwy"] = 0.0;
  */

  /*
    joint_position_map["r_camhand_joint"] = 0.0;
    joint_position_map["r_f0_base"] = 0.0;
    joint_position_map["r_f0_j0"] = 0.0;
    joint_position_map["r_f0_j1"] = 0.0;
    joint_position_map["r_f0_j2"] = 0.0;
    joint_position_map["r_f0_fixed_accel"] = 0.0;
    joint_position_map["r_f0_1_accel"] = 0.0;
    joint_position_map["r_f0_2_accel"] = 0.0;
    joint_position_map["r_f1_base"] = 0.0;
    joint_position_map["r_f1_j0"] = 0.0;
    joint_position_map["r_f1_j1"] = 0.0;
    joint_position_map["r_f1_j2"] = 0.0;
    joint_position_map["r_f1_fixed_accel"] = 0.0;
    joint_position_map["r_f1_1_accel"] = 0.0;
    joint_position_map["r_f1_2_accel"] = 0.0;
    joint_position_map["r_f2_base"] = 0.0;
    joint_position_map["r_f2_j0"] = 0.0;
    joint_position_map["r_f2_j1"] = 0.0;
    joint_position_map["r_f2_j2"] = 0.0;
    joint_position_map["r_f2_fixed_accel"] = 0.0;
    joint_position_map["r_f2_1_accel"] = 0.0;
    joint_position_map["r_f2_2_accel"] = 0.0;
    joint_position_map["r_f3_base"] = 0.0;
    joint_position_map["r_f3_j0"] = 0.0;
    joint_position_map["r_f3_j1"] = 0.0;
    joint_position_map["r_f3_j2"] = 0.0;
    joint_position_map["r_f3_fixed_accel"] = 0.0;
    joint_position_map["r_f3_1_accel"] = 0.0;
    joint_position_map["r_f3_2_accel"] = 0.0;

    joint_position_map["l_camhand_joint"] = 0.0;
    joint_position_map["l_f0_base"] = 0.0;
    joint_position_map["l_f0_j0"] = 0.0;
    joint_position_map["l_f0_j1"] = 0.0;
    joint_position_map["l_f0_j2"] = 0.0;
    joint_position_map["l_f0_fixed_accel"] = 0.0;
    joint_position_map["l_f0_1_accel"] = 0.0;
    joint_position_map["l_f0_2_accel"] = 0.0;
    joint_position_map["l_f1_base"] = 0.0;
    joint_position_map["l_f1_j0"] = 0.0;
    joint_position_map["l_f1_j1"] = 0.0;
    joint_position_map["l_f1_j2"] = 0.0;
    joint_position_map["l_f1_fixed_accel"] = 0.0;
    joint_position_map["l_f1_1_accel"] = 0.0;
    joint_position_map["l_f1_2_accel"] = 0.0;
    joint_position_map["l_f2_base"] = 0.0;
    joint_position_map["l_f2_j0"] = 0.0;
    joint_position_map["l_f2_j1"] = 0.0;
    joint_position_map["l_f2_j2"] = 0.0;
    joint_position_map["l_f2_fixed_accel"] = 0.0;
    joint_position_map["l_f2_1_accel"] = 0.0;
    joint_position_map["l_f2_2_accel"] = 0.0;
    joint_position_map["l_f3_base"] = 0.0;
    joint_position_map["l_f3_j0"] = 0.0;
    joint_position_map["l_f3_j1"] = 0.0;
    joint_position_map["l_f3_j2"] = 0.0;
    joint_position_map["l_f3_fixed_accel"] = 0.0;
    joint_position_map["l_f3_1_accel"] = 0.0;
    joint_position_map["l_f3_2_accel"] = 0.0;
  */

    this->model_->SetJointPositions(joint_position_map);

    // math::Pose pose(2, 1, 1.5, 0, 0, 0);
    // this->model_->SetLinkWorldPose(pose, "pelvis");
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCRobotPlugin)
}
