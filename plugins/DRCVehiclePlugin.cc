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
 * Desc: Plugin for vehicle control.
 * Author: John Hsu and Steve Peters
 * Date: November 2012
 */

#include <math.h>
#include "DRCVehiclePlugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/SphereShape.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehiclePlugin::DRCVehiclePlugin()
{
  this->keyState = ON;
  this->directionState = FORWARD;
  this->gasPedalCmd = 0;
  this->brakePedalCmd = 0;
  this->handWheelCmd = 0;
  this->handBrakeCmd = 1;
  this->flWheelCmd = 0;
  this->frWheelCmd = 0;
  this->blWheelCmd = 0;
  this->brWheelCmd = 0;
  this->flWheelSteeringCmd = 0;
  this->frWheelSteeringCmd = 0;

  /// \TODO: get this from model
  this->wheelRadius = 0.1;
  this->flWheelRadius = 0.1;
  this->frWheelRadius = 0.1;
  this->blWheelRadius = 0.1;
  this->brWheelRadius = 0.1;
  this->pedalForce = 10;
  this->handWheelForce = 1;
  this->handBrakeForce = 10;
  this->steeredWheelForce = 200;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehiclePlugin::~DRCVehiclePlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize
void DRCVehiclePlugin::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetVehicleState(double _handWheelPosition,
                                       double _gasPedalPosition,
                                       double _brakePedalPosition,
                                       double _handBrakePosition,
                                   DRCVehiclePlugin::KeyType _key,
                                   DRCVehiclePlugin::DirectionType _direction)
{
  // This function isn't currently looking at joint limits.
  this->handWheelCmd = _handWheelPosition;
  this->handBrakeCmd = _handBrakePosition;
  this->gasPedalCmd = _gasPedalPosition;
  this->brakePedalCmd = _brakePedalPosition;
  this->directionState = _direction;
  this->keyState = _key;
}

////////////////////////////////////////////////////////////////////////////////
DRCVehiclePlugin::DirectionType DRCVehiclePlugin::GetDirectionState()
{
  return this->directionState;
}

////////////////////////////////////////////////////////////////////////////////
DRCVehiclePlugin::KeyType DRCVehiclePlugin::GetKeyState()
{
  return this->keyState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetDirectionState(
        DRCVehiclePlugin::DirectionType _direction)
{
  this->directionState = _direction;
  if (_direction == NEUTRAL && this->keyState == ON_FR)
    this->keyState = ON;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOff()
{
  this->keyState = OFF;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOn()
{
  if (this->directionState == NEUTRAL)
    this->keyState = ON;
  else
    this->keyState = ON_FR;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasTorqueMultiplier()
{
  if (this->keyState == ON)
  {
    if (this->directionState == FORWARD)
      return 1.0;
    else if (this->directionState == REVERSE)
      return -1.0;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandBrakeState(double _position)
{
  double min, max;
  this->GetHandBrakeLimits(min, max);
  this->handBrakeCmd = this->Saturate(_position, min, max);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandBrakeLimits(double &_min, double &_max)
{
  this->handBrakeJoint->SetHighStop(0, _max);
  this->handBrakeJoint->SetLowStop(0, _min);
  this->handBrakeHigh  = this->handBrakeJoint->GetHighStop(0).Radian();
  this->handBrakeLow   = this->handBrakeJoint->GetLowStop(0).Radian();
  this->handBrakeRange   = this->handBrakeHigh - this->handBrakeLow;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandBrakeLimits(double &_min, double &_max)
{
  _max = this->handBrakeJoint->GetHighStop(0).Radian();
  _min = this->handBrakeJoint->GetLowStop(0).Radian();
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandBrakeState()
{
  return this->handBrakeState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelState(double _position)
{
  math::Angle min, max;
  this->GetHandWheelLimits(min, max);
  this->handWheelCmd = this->Saturate(_position, min.Radian(), max.Radian());
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelLimits(const math::Angle &_min,
                                          const math::Angle &_max)
{
  this->handWheelJoint->SetHighStop(0, _max);
  this->handWheelJoint->SetLowStop(0, _min);
  this->UpdateHandWheelRatio();
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandWheelLimits(math::Angle &_min, math::Angle &_max)
{
  _max = this->handWheelJoint->GetHighStop(0);
  _min = this->handWheelJoint->GetLowStop(0);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelState()
{
  return this->handWheelState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->handWheelHigh  = this->handWheelJoint->GetHighStop(0).Radian();
  this->handWheelLow   = this->handWheelJoint->GetLowStop(0).Radian();
  this->handWheelRange = this->handWheelHigh - this->handWheelLow;
  double high = std::min(this->flWheelSteeringJoint->GetHighStop(0).Radian(),
                         this->frWheelSteeringJoint->GetHighStop(0).Radian());
  double low = std::max(this->flWheelSteeringJoint->GetLowStop(0).Radian(),
                        this->frWheelSteeringJoint->GetLowStop(0).Radian());
  this->tireAngleRange = std::min( abs(high), abs(low) );

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = this->tireAngleRange / this->handWheelRange;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelRatio()
{
  return this->steeringRatio;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelState(double _position)
{
  this->SetHandWheelState(_position / this->steeringRatio);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelLimits(const math::Angle &_min,
                                         const math::Angle &_max)
{
  this->flWheelSteeringJoint->SetHighStop(0, _max);
  this->flWheelSteeringJoint->SetLowStop(0, _min);
  this->frWheelSteeringJoint->SetHighStop(0, _max);
  this->frWheelSteeringJoint->SetLowStop(0, _min);
  this->UpdateHandWheelRatio();
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetSteeredWheelState()
{
    return 0.5*(flSteeringState + frSteeringState);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetSteeredWheelLimits(math::Angle &_min, math::Angle &_max)
{
  _max = 0.5 * (this->flWheelSteeringJoint->GetHighStop(0).Radian() +
                this->frWheelSteeringJoint->GetHighStop(0).Radian());
  _max = 0.5 * (this->flWheelSteeringJoint->GetLowStop(0).Radian() +
                this->frWheelSteeringJoint->GetLowStop(0).Radian());
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalState(double _position)
{
  double min, max;
  this->GetGasPedalLimits(min, max);
  this->gasPedalCmd = this->Saturate(_position, min, max);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalLimits(double _min, double _max)
{
  this->gasPedalJoint->SetHighStop(0, _max);
  this->gasPedalJoint->SetLowStop(0, _min);
  this->gasPedalHigh  = this->gasPedalJoint->GetHighStop(0).Radian();
  this->gasPedalLow   = this->gasPedalJoint->GetLowStop(0).Radian();
  this->gasPedalRange   = this->gasPedalHigh - this->gasPedalLow;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetGasPedalLimits(double &_min, double &_max)
{
  _max = this->gasPedalJoint->GetHighStop(0).Radian();
  _min = this->gasPedalJoint->GetLowStop(0).Radian();
}

/// Returns the gas pedal position in meters.
////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalState()
{
  return this->gasPedalState;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalPercent()
{
  double min, max;
  this->GetGasPedalLimits(min, max);
  return (this->gasPedalState - min) / (max-min);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalPercent()
{
  double min, max;
  this->GetBrakePedalLimits(min, max);
  return (this->brakePedalState - min) / (max-min);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandBrakePercent()
{
  double min, max;
  this->GetHandBrakeLimits(min, max);
  return (this->handBrakeState - min) / (max-min);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalState(double _position)
{
  double min, max;
  this->GetBrakePedalLimits(min, max);
  this->brakePedalCmd = this->Saturate(_position, min, max);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalLimits(double _min, double _max)
{
  this->brakePedalJoint->SetHighStop(0, _max);
  this->brakePedalJoint->SetLowStop(0, _min);
  this->brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  this->brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  this->brakePedalRange = this->brakePedalHigh - this->brakePedalLow;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetBrakePedalLimits(double &_min, double &_max)
{
  _max = this->brakePedalJoint->GetHighStop(0).Radian();
  _min = this->brakePedalJoint->GetLowStop(0).Radian();
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalState()
{
  return this->brakePedalState;
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
    + _sdf->GetValueString("gas_pedal");
  this->gasPedalJoint = this->model->GetJoint(gasPedalJointName);

  std::string brakePedalJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("brake_pedal");
  this->brakePedalJoint = this->model->GetJoint(brakePedalJointName);

  std::string handWheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("steering_wheel");
  this->handWheelJoint = this->model->GetJoint(handWheelJointName);

  std::string handBrakeJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("hand_brake");
  this->handBrakeJoint = this->model->GetJoint(handBrakeJointName);

  std::string flWheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("front_left_wheel");
  this->flWheelJoint = this->model->GetJoint(flWheelJointName);

  std::string frWheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("front_right_wheel");
  this->frWheelJoint = this->model->GetJoint(frWheelJointName);

  std::string blWheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("back_left_wheel");
  this->blWheelJoint = this->model->GetJoint(blWheelJointName);

  std::string brWheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("back_right_wheel");
  this->brWheelJoint = this->model->GetJoint(brWheelJointName);

  std::string flWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("front_left_wheel_steering");
  this->flWheelSteeringJoint = this->model->GetJoint(flWheelSteeringJointName);

  std::string frWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("front_right_wheel_steering");
  this->frWheelSteeringJoint = this->model->GetJoint(frWheelSteeringJointName);

  this->gasPedalHigh  = this->gasPedalJoint->GetHighStop(0).Radian();
  this->gasPedalLow   = this->gasPedalJoint->GetLowStop(0).Radian();
  this->gasPedalRange   = this->gasPedalHigh - this->gasPedalLow;
  this->brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  this->brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  this->brakePedalRange   = this->brakePedalHigh - this->brakePedalLow;


  // get some vehicle parameters
  this->frontTorque = _sdf->GetValueDouble("front_torque");
  this->backTorque = _sdf->GetValueDouble("back_torque");
  this->frontBrakeTorque = _sdf->GetValueDouble("front_brake_torque");
  this->backBrakeTorque = _sdf->GetValueDouble("back_brake_torque");
  this->maxSpeed = _sdf->GetValueDouble("max_speed");
  this->aeroLoad = _sdf->GetValueDouble("aero_load");

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id=0;
  this->flWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->flWheelJoint->GetChild()->GetCollision(id));
  this->frWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->frWheelJoint->GetChild()->GetCollision(id));
  this->blWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->blWheelJoint->GetChild()->GetCollision(id));
  this->brWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->brWheelJoint->GetChild()->GetCollision(id));
  //gzerr << this->flWheelRadius << " " << this->frWheelRadius << " "
  //      << this->blWheelRadius << " " << this->brWheelRadius << "\n";

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  math::Vector3 flCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->flWheelJoint->GetChild(), id);
  math::Vector3 frCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->frWheelJoint->GetChild(), id);
  math::Vector3 blCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->blWheelJoint->GetChild(), id);
  math::Vector3 brCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->brWheelJoint->GetChild(), id);
  // track widths are computed first
  math::Vector3 vec3 = flCenterPos - frCenterPos;
  frontTrackWidth = vec3.GetLength();
  vec3 = flCenterPos - frCenterPos;
  backTrackWidth = vec3.GetLength();
  // to compute wheelbase, first position of axle centers are computed
  math::Vector3 frontAxlePos = (flCenterPos + frCenterPos) / 2;
  math::Vector3 backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  wheelbaseLength = vec3.GetLength();
  //gzerr << wheelbaseLength << " " << frontTrackWidth << " " << backTrackWidth << "\n";

  // initialize controllers for car
  /// \TODO: move PID parameters into SDF
  this->gasPedalPID.Init(200, 0, 3, 10, -10,
                         this->pedalForce, -this->pedalForce);
  this->brakePedalPID.Init(200, 0, 3, 10, -10,
                         this->pedalForce, -this->pedalForce);
  this->handWheelPID.Init(200, 0, 30.0, 5.0, -5.0,
                         this->handWheelForce, -this->handWheelForce);
  this->handBrakePID.Init(30, 0, 3.0, 5.0, -5.0,
                         this->handBrakeForce, -this->handBrakeForce);
  this->flWheelSteeringPID.Init(5000, 0, 500, 50, -50,
                         this->steeredWheelForce, -this->steeredWheelForce);
  this->frWheelSteeringPID.Init(5000, 0, 500, 50, -50,
                         this->steeredWheelForce, -this->steeredWheelForce);

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));

  this->lastTime = this->world->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCVehiclePlugin::UpdateStates()
{
  this->handWheelState = this->handWheelJoint->GetAngle(0).Radian();
  this->handBrakeState = this->handBrakeJoint->GetAngle(0).Radian();
  this->brakePedalState = this->brakePedalJoint->GetAngle(0).Radian();
  this->gasPedalState = this->gasPedalJoint->GetAngle(0).Radian();
  this->flSteeringState = this->flWheelSteeringJoint->GetAngle(0).Radian();
  this->frSteeringState = this->frWheelSteeringJoint->GetAngle(0).Radian();

  this->flWheelState = this->flWheelJoint->GetVelocity(0);
  this->frWheelState = this->frWheelJoint->GetVelocity(0);
  this->blWheelState = this->blWheelJoint->GetVelocity(0);
  this->brWheelState = this->brWheelJoint->GetVelocity(0);

  math::Vector3 linVel = this->model->GetRelativeLinearVel();
  math::Vector3 angVel = this->model->GetRelativeAngularVel();

  common::Time curTime = this->world->GetSimTime();
  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    // PID (position) steering
    double steerError = this->handWheelState - this->handWheelCmd;
    double steerCmd = this->handWheelPID.Update(steerError, dt);
    this->handWheelJoint->SetForce(0, steerCmd);

    // PID (position) hand brake
    double handBrakeError = this->handBrakeState - this->handBrakeCmd;
    double handBrakeCmd = this->handBrakePID.Update(handBrakeError, dt);
    this->handBrakeJoint->SetForce(0, handBrakeCmd);

    // PID (position) gas pedal
    double gasError = this->gasPedalState - this->gasPedalCmd;
    double gasCmd = this->gasPedalPID.Update(gasError, dt);
    this->gasPedalJoint->SetForce(0, gasCmd);

    // PID (position) brake pedal
    double brakeError = this->brakePedalState - this->brakePedalCmd;
    double brakeCmd = this->brakePedalPID.Update(brakeError, dt);
    this->brakePedalJoint->SetForce(0, brakeCmd);

    // PID (position) steering joints based on steering position
    // Ackermann steering geometry here
    //  \TODO provide documentation for these equations
    double tanSteer = tan(this->handWheelState * this->steeringRatio);
    this->flWheelSteeringCmd = atan2(tanSteer,
        1 - frontTrackWidth/2/wheelbaseLength * tanSteer);
    this->frWheelSteeringCmd = atan2(tanSteer,
        1 + frontTrackWidth/2/wheelbaseLength * tanSteer);
    //this->flWheelSteeringCmd = this->handWheelState * this->steeringRatio;
    //this->frWheelSteeringCmd = this->handWheelState * this->steeringRatio;

    double flwsError =  this->flSteeringState - this->flWheelSteeringCmd;
    double flwsCmd = this->flWheelSteeringPID.Update(flwsError, dt);
    this->flWheelSteeringJoint->SetForce(0, flwsCmd);

    double frwsError = this->frSteeringState - this->frWheelSteeringCmd;
    double frwsCmd = this->frWheelSteeringPID.Update(frwsError, dt);
    this->frWheelSteeringJoint->SetForce(0, frwsCmd);

    // Let SDF parameters specify front/rear/all-wheel drive.

    // Gas pedal torque.
    // Map gas torques to individual wheels.
    // Cut off gas torque at a given wheel if max speed is exceeded.
    // Use directionState to determine direction of applied torque.
    // Note that definition of DirectionType allows multiplication to determine
    // torque direction.
    double gasPercent = this->GetGasPedalPercent();
    double gasMultiplier = this->GetGasTorqueMultiplier();
    double flGasTorque=0, frGasTorque=0, blGasTorque=0, brGasTorque=0;
    // Apply equal torque at left and right wheels, which is an implicit model
    // of the differential.
    if (abs(this->flWheelState * this->flWheelRadius) < this->maxSpeed)
      flGasTorque = gasPercent*this->frontTorque * gasMultiplier;
    if (abs(this->frWheelState * this->frWheelRadius) < this->maxSpeed)
      frGasTorque = gasPercent*this->frontTorque * gasMultiplier;
    if (abs(this->blWheelState * this->blWheelRadius) < this->maxSpeed)
      blGasTorque = gasPercent*this->backTorque * gasMultiplier;
    if (abs(this->brWheelState * this->brWheelRadius) < this->maxSpeed)
      brGasTorque = gasPercent*this->backTorque * gasMultiplier;

    // Brake pedal, hand-brake torque.
    // Compute percents and add together, saturating at 100%
    double brakePercent = this->GetBrakePedalPercent()
      + this->GetHandBrakePercent();
    if (brakePercent > 1) brakePercent = 1;
    // Map brake torques to individual wheels.
    // Apply brake torque in opposition to wheel spin direction.
    double flBrakeTorque, frBrakeTorque, blBrakeTorque, brBrakeTorque;
    flBrakeTorque = -copysign(brakePercent*this->frontBrakeTorque,
      this->flWheelState);
    frBrakeTorque = -copysign(brakePercent*this->frontBrakeTorque,
      this->frWheelState);
    blBrakeTorque = -copysign(brakePercent*this->backBrakeTorque,
      this->blWheelState);
    brBrakeTorque = -copysign(brakePercent*this->backBrakeTorque,
      this->brWheelState);

    this->flWheelJoint->SetForce(0, flGasTorque + flBrakeTorque);
    this->frWheelJoint->SetForce(0, frGasTorque + frBrakeTorque);
    this->blWheelJoint->SetForce(0, blGasTorque + blBrakeTorque);
    this->brWheelJoint->SetForce(0, brGasTorque + brBrakeTorque);

    // gzerr << "steer [" << this->handWheelState
    //       << "] range [" << this->handWheelRange
    //       << "] l [" << linVel
    //       << "] a [" << angVel
    //       << "] gas [" << this->gasPedalState
    //       << "] gas [" << gasCmd
    //       << "] brake [" << this->brakePedalState
    //       << "] brake [" << brakeCmd
    //       << "] bl gas [" << blGasTorque
    //       << "] bl brake [" << blBrakeTorque << "]\n";
    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

// limit _data to _min and _max
double DRCVehiclePlugin::Saturate(double _data, double _min, double _max)
{
  if (_data < _min)
    return _min;
  if (_data > _max)
    return _max;
  return _data;
}

// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double DRCVehiclePlugin::get_collision_radius(physics::CollisionPtr _coll)
{
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

math::Vector3 DRCVehiclePlugin::get_collision_position(physics::LinkPtr _link,
                                                       unsigned int id)
{
  math::Pose pose = _link->GetCollision(id)->GetWorldPose();
  return pose.pos;
}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}

