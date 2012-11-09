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

#include <math.h>
#include "DRCVehiclePlugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/SphereShape.hh"

double get_collision_radius(gazebo::physics::CollisionPtr _collision);
gazebo::math::Vector3 get_collision_position(gazebo::physics::LinkPtr _link, unsigned int id);

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehiclePlugin::DRCVehiclePlugin()
{
  this->gasPedalCmd = 0;
  this->brakePedalCmd = 0.5;
  this->handwheelCmd = 0;
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
  this->handwheelForce = 1;
  this->steeredWheelForce = 200;
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
void DRCVehiclePlugin::SetVehicleState(double _handwheelPosition,
                                       double _gasPedalPosition,
                                       double _brakePedalPosition)
{
  this->handwheelCmd = _handwheelPosition;
  this->gasPedalCmd = _gasPedalPosition;
  this->brakePedalCmd = _brakePedalPosition;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandwheelState(double _position)
{
  this->handwheelCmd = _position;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandwheelLimits(const math::Angle &_min,
                                              const math::Angle &_max)
{
  this->handwheelJoint->SetHighStop(0, _max);
  this->handwheelJoint->SetLowStop(0, _min);
  this->UpdateHandwheelRatio();
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandwheelLimits(math::Angle &_min,
                                              math::Angle &_max)
{
  _max = this->handwheelJoint->GetHighStop(0);
  _min = this->handwheelJoint->GetLowStop(0);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandwheelState()
{
  return this->handwheelState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateHandwheelRatio()
{
  // The total range the steering wheel can rotate
  this->handwheelHigh  = this->handwheelJoint->GetHighStop(0).Radian();
  this->handwheelLow   = this->handwheelJoint->GetLowStop(0).Radian();
  this->handwheelRange = this->handwheelHigh - this->handwheelLow;
  double high = std::min(this->flWheelSteeringJoint->GetHighStop(0).Radian(),
                         this->frWheelSteeringJoint->GetHighStop(0).Radian());
  double low = std::max(this->flWheelSteeringJoint->GetLowStop(0).Radian(),
                        this->frWheelSteeringJoint->GetLowStop(0).Radian());
  this->tireAngleRange = std::min( abs(high), abs(low) );

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = this->tireAngleRange / this->handwheelRange;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandwheelRatio()
{
  return this->steeringRatio;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelState(double _position)
{
  this->SetHandwheelState(_position / this->steeringRatio);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelLimits(const math::Angle &_min,
                                         const math::Angle &_max)
{
  this->flWheelSteeringJoint->SetHighStop(0, _max);
  this->flWheelSteeringJoint->SetLowStop(0, _min);
  this->frWheelSteeringJoint->SetHighStop(0, _max);
  this->frWheelSteeringJoint->SetLowStop(0, _min);
  this->UpdateHandwheelRatio();
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
  this->gasPedalCmd = _position;
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
void DRCVehiclePlugin::SetBrakePedalState(double _position)
{
  this->brakePedalCmd = _position;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalLimits(double _min, double _max)
{
  this->brakePedalJoint->SetHighStop(0, _max);
  this->brakePedalJoint->SetLowStop(0, _min);
  this->brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  this->brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  this->brakePedalRange   = this->brakePedalHigh - this->brakePedalLow;
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

  std::string handwheelJointName = this->model->GetName() + "::"
    + _sdf->GetValueString("steering_wheel");
  this->handwheelJoint = this->model->GetJoint(handwheelJointName);

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

  this->UpdateHandwheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id=0;
  this->flWheelRadius = get_collision_radius(this->flWheelJoint->GetChild()->GetCollision(id));
  this->frWheelRadius = get_collision_radius(this->frWheelJoint->GetChild()->GetCollision(id));
  this->blWheelRadius = get_collision_radius(this->blWheelJoint->GetChild()->GetCollision(id));
  this->brWheelRadius = get_collision_radius(this->brWheelJoint->GetChild()->GetCollision(id));
  //gzerr << this->flWheelRadius << " " << this->frWheelRadius << " "
  //      << this->blWheelRadius << " " << this->brWheelRadius << "\n";

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  math::Vector3 flCenterPos = get_collision_position(this->flWheelJoint->GetChild(), id);
  math::Vector3 frCenterPos = get_collision_position(this->frWheelJoint->GetChild(), id);
  math::Vector3 blCenterPos = get_collision_position(this->blWheelJoint->GetChild(), id);
  math::Vector3 brCenterPos = get_collision_position(this->brWheelJoint->GetChild(), id);
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
  this->gasPedalPID.Init(200, 1, 3, 10, -10,
                         this->pedalForce, -this->pedalForce);
  this->brakePedalPID.Init(200, 1, 3, 10, -10,
                           this->pedalForce, -this->pedalForce);
  this->handwheelPID.Init(30, 0.1, 3.0, 5.0, -5.0,
                           this->handwheelForce, -this->handwheelForce);
  this->flWheelSteeringPID.Init(500, 1, 10, 50, -50,
                                this->steeredWheelForce, -this->steeredWheelForce);
  this->frWheelSteeringPID.Init(500, 1, 10, 50, -50,
                                this->steeredWheelForce, -this->steeredWheelForce);

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));

  this->lastTime = this->world->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCVehiclePlugin::UpdateStates()
{
  this->handwheelState = this->handwheelJoint->GetAngle(0).Radian();
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
    double steerError = this->handwheelState - this->handwheelCmd;
    double steerCmd = this->handwheelPID.Update(steerError, dt);
    this->handwheelJoint->SetForce(0, steerCmd);

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
    double tanSteeredWheelState = tan(this->handwheelState * this->steeringRatio);
    this->flWheelSteeringCmd = atan2(tanSteeredWheelState, 1 - frontTrackWidth/2/wheelbaseLength * tanSteeredWheelState);
    this->frWheelSteeringCmd = atan2(tanSteeredWheelState, 1 + frontTrackWidth/2/wheelbaseLength * tanSteeredWheelState);
    //this->flWheelSteeringCmd = this->handwheelState * this->steeringRatio;
    //this->frWheelSteeringCmd = this->handwheelState * this->steeringRatio;

    double flwsError =  this->flSteeringState - this->flWheelSteeringCmd;
    double flwsCmd = this->flWheelSteeringPID.Update(flwsError, dt);
    this->flWheelSteeringJoint->SetForce(0, flwsCmd);

    double frwsError = this->frSteeringState - this->frWheelSteeringCmd;
    double frwsCmd = this->frWheelSteeringPID.Update(frwsError, dt);
    this->frWheelSteeringJoint->SetForce(0, frwsCmd);

    // PID (wheel torque) front wheels based on gas position and velocity
    double frontTorqueCmd;
    if (abs(this->blWheelState * this->wheelRadius * 2.0) > this->maxSpeed)
      frontTorqueCmd = 0;
    else
      frontTorqueCmd = this->frontTorque *
                       (this->gasPedalState / this->gasPedalRange);

    this->flWheelJoint->SetForce(0, frontTorqueCmd);
    this->frWheelJoint->SetForce(0, frontTorqueCmd);

    // PID (wheel torque) back wheels based on brake position and velocity
    double backTorqueCmd;
    double vel = this->blWheelState * this->wheelRadius * 2.0;
    if (abs(vel) > this->maxSpeed)
    {
      backTorqueCmd = 0;
    }
    else
    {
      backTorqueCmd = this->backTorque *
                       (this->gasPedalState / this->gasPedalRange);
    }
    /// apply brake
    backTorqueCmd -= copysign(this->backBrakeTorque *
              (this->brakePedalState / this->brakePedalRange), vel);

    this->blWheelJoint->SetForce(0, backTorqueCmd);
    this->brWheelJoint->SetForce(0, backTorqueCmd);

    // gzerr << "steer [" << this->handwheelState
    //       << "] range [" << this->handwheelRange
    //       << "] l [" << linVel
    //       << "] a [" << angVel
    //       << "] gas [" << this->gasPedalState
    //       << "] gas [" << gasCmd
    //       << "] brake [" << this->brakePedalState
    //       << "] brake [" << brakeCmd
    //       << "] torque [" << backTorqueCmd << "]\n";
    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}

// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double get_collision_radius(gazebo::physics::CollisionPtr _collision)
{
  if (_collision->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    gazebo::physics::CylinderShape *cyl = static_cast<gazebo::physics::CylinderShape*>(_collision->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_collision->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE))
  {
    gazebo::physics::SphereShape *sph = static_cast<gazebo::physics::SphereShape*>(_collision->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

gazebo::math::Vector3 get_collision_position(gazebo::physics::LinkPtr _link, unsigned int id)
{
  gazebo::math::Pose pose = _link->GetCollision(id)->GetWorldPose();
  return pose.pos;
}
