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

#include "drcsim_gazebo_plugins/DRCBuildingPlugin.hh"

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
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
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

  this->doorLink = this->model->GetLink(_sdf->Get<std::string>("door_link"));
  if (!this->doorLink)
  {
    gzerr << "<door_link>" << _sdf->Get<std::string>("door_link")
          << "<door_link> does not exist\n";
    return;
  }

  this->doorJoint = this->model->GetJoint(_sdf->Get<std::string>("door_joint"));
  if (!this->doorJoint)
  {
    gzerr << "<door_joint>" << _sdf->Get<std::string>("door_joint")
          << "<door_joint> does not exist\n";
    return;
  }

  this->handleJoint = this->model->GetJoint(
    _sdf->Get<std::string>("handle_joint"));
  if (!this->handleJoint)
  {
    gzerr << "<handle_joint>" << _sdf->Get<std::string>("handle_joint")
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
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
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
#if GAZEBO_MAJOR_VERSION >= 4
      this->doorJoint->SetPosition(0, 0);
#else
      this->doorJoint->SetAngle(0, 0);
#endif
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
