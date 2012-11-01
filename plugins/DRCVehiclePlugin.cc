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

#include "DRCVehiclePlugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehiclePlugin::DRCVehiclePlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehiclePlugin::~DRCVehiclePlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCVehiclePlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  // Get joints
  std::string gasPedalJointName = this->model->GetName() + "::"
    + _sdf->GetElement("gas_pedal")->GetValueString();
  this->gasPedalJoint = this->model->GetJoint(gasPedalJointName);

  std::string brakePedalJointName = this->model->GetName() + "::"
    + _sdf->GetElement("brake_pedal")->GetValueString();
  this->brakePedalJoint = this->model->GetJoint(brakePedalJointName);

  std::string steeringWheelJointName = this->model->GetName() + "::"
    + _sdf->GetElement("steering_wheel")->GetValueString();
  this->steeringWheelJoint = this->model->GetJoint(steeringWheelJointName);

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));
}

// Set DRC Robot feet placement
void DRCVehiclePlugin::SetSteeringWheelState(math::Angle _position)
{
  physics::LinkPtr steering_wheel = this->model->GetLink("steering_wheel");
  physics::LinkPtr fl_wheel = this->model->GetLink("front_left_wheel");
  physics::LinkPtr fr_wheel = this->model->GetLink("front_right_wheel");
  physics::LinkPtr bl_wheel = this->model->GetLink("back_left_wheel");
  physics::LinkPtr br_wheel = this->model->GetLink("back_right_wheel");

}


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCVehiclePlugin::UpdateStates()
{
  common::Time cur_time = this->world->GetSimTime();

}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}
