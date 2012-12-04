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
  this->doorCmd = 0;
  this->handleCmd = 0;
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
  if (!this->doorLink)
  {
    gzerr << "<door_link>" << _sdf->GetValueString("door_link")
          << "<door_link> does not exist\n";
    return;
  }

  this->doorJoint = this->model->GetJoint(_sdf->GetValueString("door_joint"));
  if (!this->doorJoint)
  {
    gzerr << "<door_joint>" << _sdf->GetValueString("door_joint")
          << "<door_joint> does not exist\n";
    return;
  }

  this->handleJoint = this->model->GetJoint(
    _sdf->GetValueString("handle_joint"));
  if (!this->handleJoint)
  {
    gzerr << "<handle_joint>" << _sdf->GetValueString("handle_joint")
          << "<handle_joint> does not exist\n";
    return;
  }

  this->doorJoint->SetHighStop(0, 0);
  this->doorJoint->SetLowStop(0, 0);

  this->doorPID.Init(200, 1, 20, 10, -10, 50, -50);
  this->handlePID.Init(80, 1, 1, 3, -3, 5, -5);
  this->lastTime = this->world->GetSimTime();

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
  common::Time curTime = this->world->GetSimTime();
  this->doorState = this->doorJoint->GetAngle(0).Radian();
  this->handleState = this->handleJoint->GetAngle(0).Radian();

  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    // PID (position) door
    double doorError = this->doorState - this->doorCmd;
    double doorCmd = this->doorPID.Update(doorError, dt);
    this->doorJoint->SetForce(0, doorCmd);

    // PID (position) handle
    double handleError = this->handleState - this->handleCmd;
    double handleCmd = this->handlePID.Update(handleError, dt);
    this->handleJoint->SetForce(0, handleCmd);

    // simulate door latch/lock
    if ((fabs(this->handleState) < 0.02) && (fabs(this->doorState)   < 0.02))
    {
      this->doorJoint->SetHighStop(0, 0);
      this->doorJoint->SetLowStop(0, 0);
      this->doorJoint->SetAngle(0, 0);
    }
    else
    {
      this->doorJoint->SetHighStop(0, 1.5708);
      this->doorJoint->SetLowStop(0, -1.5708);
    }

    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCBuildingPlugin)
}
