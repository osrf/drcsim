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

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehiclePlugin::DRCVehiclePlugin()
{
  this->gasPedalCmd = 0;
  this->brakePedalCmd = 0;
  this->handWheelCmd = 0;
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
  this->steeredWheelForce = 200;

  this->rosPublishPeriod = common::Time(1.0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehiclePlugin::~DRCVehiclePlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
  event::Events::DisconnectWorldUpdateStart(this->ros_publish_connection_);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize
void DRCVehiclePlugin::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetVehicleState(double _handWheelPosition,
                                       double _gasPedalPosition,
                                       double _brakePedalPosition)
{
  this->handWheelCmd = _handWheelPosition;
  this->gasPedalCmd = _gasPedalPosition;
  this->brakePedalCmd = _brakePedalPosition;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelState(double _position)
{
  this->handWheelCmd = _position;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  this->handWheelCmd = (double)_msg->data;
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
void DRCVehiclePlugin::GetHandWheelLimits(math::Angle &_min,
                                              math::Angle &_max)
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
  this->gasPedalCmd = _position;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalState(const std_msgs::Float64::ConstPtr &_msg)
{
  this->gasPedalCmd = (double)_msg->data;
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
void DRCVehiclePlugin::SetBrakePedalState(const std_msgs::Float64::ConstPtr
    &_msg)
{
  this->brakePedalCmd = (double)_msg->data;
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
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

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
  this->gasPedalPID.Init(200, 1, 3, 10, -10,
                         this->pedalForce, -this->pedalForce);
  this->brakePedalPID.Init(200, 1, 3, 10, -10,
                         this->pedalForce, -this->pedalForce);
  this->handWheelPID.Init(30, 0.1, 3.0, 5.0, -5.0,
                         this->handWheelForce, -this->handWheelForce);
  this->flWheelSteeringPID.Init(500, 1, 10, 50, -50,
                         this->steeredWheelForce, -this->steeredWheelForce);
  this->frWheelSteeringPID.Init(500, 1, 10, 50, -50,
                         this->steeredWheelForce, -this->steeredWheelForce);

  ros::SubscribeOptions hand_wheel_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/" + this->model->GetName() + "/hand_wheel/cmd", 100,
    boost::bind( static_cast<void (DRCVehiclePlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehiclePlugin::SetHandWheelState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->hand_wheel_cmd_sub_ = this->rosnode_->subscribe(hand_wheel_cmd_so);

  ros::SubscribeOptions gas_pedal_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/" + this->model->GetName() + "/gas_pedal/cmd", 100,
    boost::bind( static_cast<void (DRCVehiclePlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehiclePlugin::SetGasPedalState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->gas_pedal_cmd_sub_ = this->rosnode_->subscribe(gas_pedal_cmd_so);

  ros::SubscribeOptions brake_pedal_cmd_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "/" + this->model->GetName() + "/brake_pedal/cmd", 100,
    boost::bind( static_cast<void (DRCVehiclePlugin::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &DRCVehiclePlugin::SetBrakePedalState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->brake_pedal_cmd_sub_ = this->rosnode_->subscribe(brake_pedal_cmd_so);

  this->hand_wheel_state_pub_ = this->rosnode_->advertise<std_msgs::Float64>(
    "/" + this->model->GetName() + "/hand_wheel/state",10);
  this->gas_pedal_state_pub_ = this->rosnode_->advertise<std_msgs::Float64>(
    "/" + this->model->GetName() + "/gas_pedal/state",10);
  this->brake_pedal_state_pub_ = this->rosnode_->advertise<std_msgs::Float64>(
    "/" + this->model->GetName() + "/brake_pedal/state",10);

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &DRCVehiclePlugin::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));
  this->ros_publish_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCVehiclePlugin::RosPublishStates, this));

  this->lastTime = this->world->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCVehiclePlugin::UpdateStates()
{
  this->handWheelState = this->handWheelJoint->GetAngle(0).Radian();
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

    // gzerr << "steer [" << this->handWheelState
    //       << "] range [" << this->handWheelRange
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

////////////////////////////////////////////////////////////////////////////////
// Returns the ROS publish period (seconds).
common::Time DRCVehiclePlugin::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}

////////////////////////////////////////////////////////////////////////////////
// Set the ROS publish frequency (Hz).
void DRCVehiclePlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    this->rosPublishPeriod = 1.0/_hz;
  else
    this->rosPublishPeriod = 0.0;  
}

////////////////////////////////////////////////////////////////////////////////
// Publish hand wheel, gas pedal, and brake pedal on ROS
void DRCVehiclePlugin::RosPublishStates()
{
  if (this->world->GetSimTime() - this->lastRosPublishTime >=
      this->rosPublishPeriod)
  {
    // Update time
    this->lastRosPublishTime = this->world->GetSimTime();
    // Publish messages
    std_msgs::Float64 msg_steer, msg_brake, msg_gas;
    msg_steer.data = GetHandWheelState();
    this->hand_wheel_state_pub_.publish(msg_steer);
    msg_brake.data = GetBrakePedalState();
    this->brake_pedal_state_pub_.publish(msg_brake);
    msg_gas.data = GetGasPedalState();
    this->gas_pedal_state_pub_.publish(msg_gas);
  }
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

void DRCVehiclePlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}

