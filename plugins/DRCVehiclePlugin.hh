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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_DRC_VEHICLE_PLUGIN_HH
#define GAZEBO_DRC_VEHICLE_PLUGIN_HH

#include <boost/thread.hpp>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/PID.hh"

#include "boost/thread/mutex.hpp"

namespace gazebo
{
  class DRCVehiclePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: DRCVehiclePlugin();

    /// \brief Destructor.
    public: virtual ~DRCVehiclePlugin();

    /// \brief Load the controller.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller.
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    private: boost::mutex update_mutex;

    /// Pointer to the update event connection.
    private: event::ConnectionPtr update_connection_;

    /// \brief Sets DRC Vehicle control inputs, the vehicle internal model 
    ///        will decide the overall motion of the vehicle.
    /// \param[in] _handWheelPosition steering wheel position in radians.
    /// \param[in] _gasPedalPosition gas pedal position in meters.
    /// \param[in] _brakePedalPosition brake pedal position in meters.
    public: void SetVehicleState(double _handWheelPosition,
                                 double _gasPedalPosition,
                                 double _brakePedalPosition);

    /// \brief Set the steering wheel angle; this will also update the front
    ///        wheel steering angle.
    /// \param[in] _position Steering wheel angle in radians.
    public: void SetHandWheelState(double _position);

    /// \brief Sets the lower and upper limits of the steering wheel angle.
    /// \param[in] _min Lower limit of steering wheel angle (radians).
    /// \param[in] _max Upper limit of steering wheel angle (radians).
    public: void SetHandWheelLimits(const math::Angle &_min,
                                    const math::Angle &_max);

    /// \brief Returns the lower and upper limits of the steering wheel angle.
    /// \param[out] _min Lower steering wheel limit (radians).
    /// \param[out] _max Upper steering wheel limit (radians).
    public: void GetHandWheelLimits(math::Angle &_min, math::Angle &_max);

    /// \brief Returns the steering wheel angle (rad).
    public: double GetHandWheelState();

    /// \brief Computes the front wheel angle / steering wheel angle ratio.
    public: void UpdateHandWheelRatio();

    /// \brief Returns the front wheel angle / steering wheel angle ratio.
    public: double GetHandWheelRatio();


    /// \brief Specify front wheel orientation in radians (Note: this sets
    /// the vehicle wheels as oppsed to the steering wheel angle set by
    /// SetHandWheelState).
    /// Zero setting results in vehicle traveling in a straight line.
    /// Positive steering angle results in a left turn in forward motion.
    /// Negative steering angle results in a right turn in forward motion.
    /// Setting front wheel steering angle will also update the
    /// handWheel steering angle.
    /// \param[in] _position Desired angle of front steered wheels in radians.
    public: void SetSteeredWheelState(double _position);

    /// \brief Sets the lower and upper limits of the steering angle (rad).
    /// \param[in] _min Lower limit of steered wheel angle (radians).
    /// \param[in] _max Upper limit of steered wheel angle (radians).
    public: void SetSteeredWheelLimits(const math::Angle &_min,
                                       const math::Angle &_max);

    /// \brief Returns the steering angle of the steered wheels (rad).
    public: double GetSteeredWheelState();

    /// \brief Returns the lower and upper limits of the steering angle
    ///        of the steered wheels (rad).
    /// \param[out] _min Lower limit of steered wheel angle (radians).
    /// \param[out] _max Upper limit of steered wheel angle (radians).
    public: void GetSteeredWheelLimits(math::Angle &_min, math::Angle &_max);

    /// \brief Specify gas pedal position in meters.
    /// \param[in] _position Desired gas pedal position in meters.
    public: void SetGasPedalState(double _position);

    /// Sets gas pedal position limits in meters.
    public: void SetGasPedalLimits(double _min, double _max);

    /// Returns gas pedal position limits in meters.
    public: void GetGasPedalLimits(double &_min, double &_max);

    /// Returns the gas pedal position in meters.
    public: double GetGasPedalState();

    /// Specify gas pedal position in meters.
    public: void SetBrakePedalState(double _position);

    /// Sets gas pedal position limits in meters.
    public: void SetBrakePedalLimits(double _min, double _max);

    /// Returns gas pedal position limits in meters.
    public: void GetBrakePedalLimits(double &_min, double &_max);

    /// Returns the gas pedal position in meters.
    public: double GetBrakePedalState();

    /// Default plugin init call.
    public: void Init();

    private: double get_collision_radius(physics::CollisionPtr _collision);
    private: math::Vector3 get_collision_position(physics::LinkPtr _link,
                                                  unsigned int id);

    private: physics::JointPtr gasPedalJoint;
    private: physics::JointPtr brakePedalJoint;
    private: physics::JointPtr handWheelJoint;
    private: physics::JointPtr flWheelJoint;
    private: physics::JointPtr frWheelJoint;
    private: physics::JointPtr blWheelJoint;
    private: physics::JointPtr brWheelJoint;
    private: physics::JointPtr flWheelSteeringJoint;
    private: physics::JointPtr frWheelSteeringJoint;

    private: double frontTorque;
    private: double backTorque;
    private: double frontBrakeTorque;
    private: double backBrakeTorque;
    private: double tireAngleRange;
    private: double maxSpeed;
    private: double aeroLoad;
    private: double steeringRatio;
    private: double pedalForce;
    private: double handWheelForce;
    private: double steeredWheelForce;

    private: double gasPedalCmd;
    private: double brakePedalCmd;
    private: double handWheelCmd;
    private: double flWheelCmd;
    private: double frWheelCmd;
    private: double blWheelCmd;
    private: double brWheelCmd;
    private: double flWheelSteeringCmd;
    private: double frWheelSteeringCmd;

    private: common::PID gasPedalPID;
    private: common::PID brakePedalPID;
    private: common::PID handWheelPID;
    private: common::PID flWheelSteeringPID;
    private: common::PID frWheelSteeringPID;

    private: common::Time lastTime;

    /// joint information from model
    private: double gasPedalHigh;
    private: double gasPedalLow;
    private: double gasPedalRange;
    private: double brakePedalHigh;
    private: double brakePedalLow;
    private: double brakePedalRange;
    private: double handWheelHigh;
    private: double handWheelLow;
    private: double handWheelRange;
    private: double wheelRadius;
    private: double flWheelRadius;
    private: double frWheelRadius;
    private: double blWheelRadius;
    private: double brWheelRadius;
    private: double wheelbaseLength;
    private: double frontTrackWidth;
    private: double backTrackWidth;

    /// state of cart
    private: double handWheelState;
    private: double flSteeringState;
    private: double frSteeringState;
    private: double gasPedalState;
    private: double brakePedalState;
    private: double flWheelState;
    private: double frWheelState;
    private: double blWheelState;
    private: double brWheelState;
  };
/** \} */
/// @}
}
#endif
