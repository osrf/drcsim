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

#include "physics/physics.h"
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
    /// \brief Constructor
    public: DRCVehiclePlugin();

    /// \brief Destructor
    public: virtual ~DRCVehiclePlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    private: boost::mutex update_mutex;

    /// Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    /// Sets DRC Vehicle control
    ///   - specify steering wheel position in radians
    ///   - specify gas pedal position in meters
    ///   - specify brake pedal position in meters
    /// The vehicle internal model will decide the overall motion
    /// of the vehicle.
    public: void SetVehicleState(math::Angle _steering_wheel_position,
                                 double _gas_pedal_position,
                                 double _brake_pedal_position);

    /// Set the steering wheel angle (rad)
    /// Setting steering wheel angle will also update the front wheel
    /// steering angle
    public: void SetSteeringWheelState(math::Angle _position);

    /// Front wheel steer angle = ratio * steering wheel angle
    public: void SetSteeringWheelRatio(double _ratio);

    /// Sets the lower and upper limits of the steering wheel angle (rad)
    public: void SetSteeringWheelLimits(math::Angle _min, math::Angle _max);

    /// Returns the lower and upper limits of the steering wheel angle (rad)
    public: void GetSteeringWheelLimits(math::Angle &_min, math::Angle &_max);

    /// Returns the steering wheel angle (rad)
    public: math::Angle GetSteeringWheelState();

    /// Returns the front wheel angle / steering wheel angle ratio
    public: double GetSteeringWheelRatio();


    /// Specify front wheel orientation in radians (Note:  this sets
    /// the vehicle wheels as oppsed to the steering wheel angle set by
    /// SetSteeringWheelState).
    /// Zero setting results in vehicle traveling in a straight line.
    /// Positive steering angle results in a left turn in forward motion.
    /// Negative steering angle results in a right turn in forward motion.
    /// Setting front wheel steering angle will also update the
    /// steering wheel angle
    public: void SetSteeringState(math::Angle _position);

    /// Sets the lower and upper limits of the steering angle (rad)
    public: void SetSteeringLimits(math::Angle _min, math::Angle _max);

    /// Returns the steering angle (rad)
    public: math::Angle GetSteeringState();

    /// Returns the lower and upper limits of the steering angle (rad)
    public: void GetSteeringLimits(math::Angle &_min, math::Angle &_max);


    /// Specify gas pedal position in meters.
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

    public: void Init();

    /// Returns the gas pedal position in meters.
    public: double GetBrakePedalState();

    private: physics::JointPtr gasPedalJoint;
    private: physics::JointPtr brakePedalJoint;
    private: physics::JointPtr steeringWheelJoint;
    private: physics::JointPtr flWheelJoint;
    private: physics::JointPtr frWheelJoint;
    private: physics::JointPtr blWheelJoint;
    private: physics::JointPtr brWheelJoint;
    private: physics::JointPtr flWheelSteeringJoint;
    private: physics::JointPtr frWheelSteeringJoint;

    private: double frontTorque;
    private: double rearTorque;
    private: double tireAngleRange;
    private: double maxSpeed;
    private: double aeroLoad;
    private: double steeringRatio;

    private: double gasPedalCmd;
    private: double brakePedalCmd;
    private: double steeringWheelCmd;
    private: double flWheelCmd;
    private: double frWheelCmd;
    private: double blWheelCmd;
    private: double brWheelCmd;
    private: double flWheelSteeringCmd;
    private: double frWheelSteeringCmd;

    private: common::PID gasPedalPID;
    private: common::PID brakePedalPID;
    private: common::PID steeringWheelPID;
    private: common::PID flWheelPID;
    private: common::PID frWheelPID;
    private: common::PID blWheelPID;
    private: common::PID brWheelPID;
    private: common::PID flWheelSteeringPID;
    private: common::PID frWheelSteeringPID;

    private: common::Time lastTime;
  };
/** \} */
/// @}
}
#endif
