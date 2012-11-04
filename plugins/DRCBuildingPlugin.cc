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

#include "DRCBuildingPlugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCBuildingPlugin::DRCBuildingPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCBuildingPlugin::~DRCBuildingPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCBuildingPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;
  this->world->EnablePhysicsEngine(true);

  this->doorLink = this->model->GetLink(_sdf->GetValueString("door_link"));
  this->doorJoint = this->model->GetJoint(_sdf->GetValueString("door_joint"));
  this->handleJoint = this->model->GetJoint(
    _sdf->GetValueString("handle_joint"));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCBuildingPlugin::UpdateStates, this));
}


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCBuildingPlugin::UpdateStates()
{
  common::Time cur_time = this->world->GetSimTime();

  std::map<std::string, double> joint_position_map;
  joint_position_map["arm_shoulder_pan_joint"] = cos(cur_time.Double());
  joint_position_map["arm_elbow_pan_joint"] = -cos(cur_time.Double());
  joint_position_map["arm_wrist_lift_joint"] = -0.35
    + 0.45*cos(0.5*cur_time.Double());
  joint_position_map["arm_wrist_roll_joint"] = -2.9*cos(3.0*cur_time.Double());

  this->model->SetJointPositions(joint_position_map);
}

GZ_REGISTER_MODEL_PLUGIN(DRCBuildingPlugin)
}
