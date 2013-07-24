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
#ifndef GAZEBO_DRC_BUILDING_PLUGIN_HH
#define GAZEBO_DRC_BUILDING_PLUGIN_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  /// \defgroup drc_plugin DRC Plugins
  /// \addtogroup drc_plugin
  /// \{
  class DRCBuildingPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DRCBuildingPlugin();

    /// \brief Destructor
    public: virtual ~DRCBuildingPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Sets DRC Building door position (rad) given door Joint name.
    ///   - zero angle means door is closed
    ///   - door hinge axis points upwards, which means
    ///     negative angle swings door counter-clockwise if view
    ///     from above.
    public: void SetDoorState(std::string _doorName, math::Angle _angle);

    /// \brief Returns DRC Building door position (rad) given door Joint name.
    /// \sa To set door state, see SetDoorState.
    public: math::Angle GetDoorState(std::string _doorName);

    private: physics::LinkPtr doorLink;
    private: physics::JointPtr doorJoint;
    private: physics::JointPtr handleJoint;

    private: common::PID doorPID;
    private: double doorState;
    private: double doorCmd;
    private: common::PID handlePID;
    private: double handleState;
    private: double handleCmd;
    private: common::Time lastTime;
  };
/// \}
}
#endif
