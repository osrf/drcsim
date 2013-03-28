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
#ifndef _GAZEBO_VRC_SCORING_PLUGIN_HH_
#define _GAZEBO_VRC_SCORING_PLUGIN_HH_

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  /// \brief A plugin that implements the VRC scoring algorithms.
  class VRCScoringPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: VRCScoringPlugin();

    /// \brief Destructor
    public: virtual ~VRCScoringPlugin();

    /// \brief Load the plugin
    /// \param[in] _world Pointer to the world.
    /// \param[in] _sdf Point the to world's SDF.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Called when the world is updated.
    /// \param[in] _info Current world information.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Update the current score.
    private: void UpdateScore();

    /// \brief Data about a gate.
    private: class Gate
             {
               /// \brief Name of the gate
               public: std::string name;

               /// \brief Center of the gate.
               public: gazebo::math::pose center;

               /// \brief Entrance of the gate.
               public: gazebo::math::pose entrance;

               /// \brief Exit of the gate.
               public: gazebo::math::pose exit;

               /// \brief Time Atlas entered the gate.
               public: gazebo::common::Time entered;

               /// \brief Time Atlas exited the gate.
               public: gazebo::common::Time exited;
             };

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Pointer to Atlas.
    private: physics::ModelPtr atlas;

    /// \brief Pointer to a Gazebo node. Used for communication.
    private: transport::NodePtr node;

    /// \brief The publisher of scoring information.
    private: transport::PublisherPtr scorePub;

    /// \brief The publisher of timing information.
    private: transport::PublisherPtr timePub;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief List of all the gates in the world. We assume that gates have
    /// the names: gate_1, gate_2, ..., gate_n.
    private: std::list<Gate> gates;

    /// \brief Time at which Atlas passed through the first gate.
    private: gazebo::comon::Time startTime;

    /// \brief The current live score.
    private: double score;

    /// \brief Pose of the atlas at the previous timestep.
    private: gazebo::math::Pose prevAtlasPose;

    private: common::Time prevTime;
  };
}
#endif
