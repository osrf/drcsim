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
  this->gasPedalCmd = 0;
  this->brakePedalCmd = 0;
  this->steeringWheelCmd = 0;
  this->flWheelCmd = 0;
  this->frWheelCmd = 0;
  this->blWheelCmd = 0;
  this->brWheelCmd = 0;
  this->flWheelSteeringCmd = 0;
  this->frWheelSteeringCmd = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehiclePlugin::~DRCVehiclePlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize
void DRCVehiclePlugin::Init()
{
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

  std::string flWheelJointName = this->model->GetName() + "::"
    + _sdf->GetElement("front_left_wheel")->GetValueString();
  this->flWheelJoint = this->model->GetJoint(flWheelJointName);

  std::string frWheelJointName = this->model->GetName() + "::"
    + _sdf->GetElement("front_right_wheel")->GetValueString();
  this->frWheelJoint = this->model->GetJoint(frWheelJointName);

  std::string blWheelJointName = this->model->GetName() + "::"
    + _sdf->GetElement("back_left_wheel")->GetValueString();
  this->blWheelJoint = this->model->GetJoint(blWheelJointName);

  std::string brWheelJointName = this->model->GetName() + "::"
    + _sdf->GetElement("back_right_wheel")->GetValueString();
  this->brWheelJoint = this->model->GetJoint(brWheelJointName);

  std::string flWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->GetElement("front_left_wheel_steering")->GetValueString();
  this->flWheelSteeringJoint = this->model->GetJoint(
    flWheelSteeringJointName);

  std::string frWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->GetElement("front_right_wheel_steering")->GetValueString();
  this->frWheelSteeringJoint = this->model->GetJoint(
    frWheelSteeringJointName);

  double flwSteeringHigh = this->flWheelSteeringJoint->GetHighStop(0).Radian();
  double flwSteeringLow = this->flWheelSteeringJoint->GetLowStop(0).Radian();
  double frwSteeringHigh = this->frWheelSteeringJoint->GetHighStop(0).Radian();
  double frwSteeringLow = this->frWheelSteeringJoint->GetLowStop(0).Radian();

  double gasPedalHigh  = this->gasPedalJoint->GetHighStop(0).Radian();
  double gasPedalLow   = this->gasPedalJoint->GetLowStop(0).Radian();
  double brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  double brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  // The total range the steering wheel can rotate
  double steeringHigh  = this->steeringWheelJoint->GetHighStop(0).Radian();
  double steeringLow   = this->steeringWheelJoint->GetLowStop(0).Radian();
  double steeringRange = steeringHigh - steeringLow;

  this->tireAngleRange = this->frWheelSteeringJoint->GetHighStop(0).Radian() -
                         this->frWheelSteeringJoint->GetLowStop(0).Radian();

  // get some vehicle parameters
  this->frontTorque = _sdf->GetElement("front_torque")->GetValueDouble();
  this->rearTorque = _sdf->GetElement("rear_torque")->GetValueDouble();
  this->maxSpeed = _sdf->GetElement("max_speed")->GetValueDouble();
  this->aeroLoad = _sdf->GetElement("aero_load")->GetValueDouble();

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // initialize controllers for car
  /// \TODO: move PID parameters into SDF
  this->gasPedalPID.Init(50, 1, 3, 10, -10, gasPedalHigh, gasPedalLow);
  this->brakePedalPID.Init(50, 1, 3, 10, -10, brakePedalHigh, brakePedalLow);
  this->steeringWheelPID.Init(20, 1, 1, 5, -5, steeringHigh, steeringLow);
  this->flWheelSteeringPID.Init(80, 1, 8, 10, -10,
                                flwSteeringHigh, flwSteeringHigh);
  this->frWheelSteeringPID.Init(80, 1, 8, 10, -10,
                                frwSteeringHigh, frwSteeringHigh);

  this->flWheelPID.Init(100, 1, 5, 10, -10,
                        this->frontTorque, -this->frontTorque);
  this->frWheelPID.Init(100, 1, 5, 10, -10,
                        this->frontTorque, -this->frontTorque);
  this->blWheelPID.Init(100, 1, 5, 10, -10,
                        this->rearTorque, -this->rearTorque);
  this->brWheelPID.Init(100, 1, 5, 10, -10,
                        this->rearTorque, -this->rearTorque);


  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));

  this->lastTime = this->world->GetSimTime();
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
  common::Time curTime = this->world->GetSimTime();
  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    double error;
    double cmd;

    // PID (position) steering
    error = this->steeringWheelJoint->GetAngle(0).Radian() -
            this->steeringWheelCmd;
    cmd = this->steeringWheelPID.Update(error, dt);
    this->steeringWheelJoint->SetForce(0, cmd);

    // PID (position) gas pedal
    error = this->gasPedalJoint->GetAngle(0).Radian() -
            this->gasPedalCmd;
    cmd = this->gasPedalPID.Update(error, dt);
    this->gasPedalJoint->SetForce(0, cmd);

    // PID (position) brake pedal
    error = this->brakePedalJoint->GetAngle(0).Radian() -
            this->brakePedalCmd;
    cmd = this->brakePedalPID.Update(error, dt);
    this->brakePedalJoint->SetForce(0, cmd);

    // PID (position) steering joints based on steering position
    this->flWheelSteeringCmd = this->steeringWheelCmd * this->steeringRatio;
    this->frWheelSteeringCmd = this->steeringWheelCmd * this->steeringRatio;

    error = this->flWheelSteeringJoint->GetAngle(0).Radian() -
            this->flWheelSteeringCmd;
    cmd = this->flWheelSteeringPID.Update(error, dt);
    this->flWheelSteeringJoint->SetForce(0, cmd);

    error = this->frWheelSteeringJoint->GetAngle(0).Radian() -
            this->frWheelSteeringCmd;
    cmd = this->frWheelSteeringPID.Update(error, dt);
    this->frWheelSteeringJoint->SetForce(0, cmd);

    // PID (wheel torque) rear wheels based on gas position and velocity
    error = this->steeringWheelJoint->GetAngle(0).Radian() -
            this->steeringWheelCmd;
    cmd = this->steeringWheelPID.Update(error, dt);
    this->steeringWheelJoint->SetForce(0, cmd);

    // PID (wheel torque) rear wheels based on brake position and velocity
    error = this->steeringWheelJoint->GetAngle(0).Radian() -
            this->steeringWheelCmd;
    cmd = this->steeringWheelPID.Update(error, dt);
    this->steeringWheelJoint->SetForce(0, cmd);

  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}
