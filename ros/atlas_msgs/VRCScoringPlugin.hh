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
#include <boost/filesystem.hpp>

#include <gazebo/math/Pose.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

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

    /// \brief Check the next gate to see if we've passed it
    private: bool CheckNextGate();

    /// \brief Check whether we've fallen
    private: bool CheckFall(const common::Time &_currTime);

    /// \brief Check whether the drill is in the bin
    private: bool CheckDrillInBin();

    /// \brief Write intermediate score data
    private: void WriteIntermediateScore(
      const gazebo::common::Time& _currTime);

    /// \brief Is this world gate-based?
    private: bool IsGateBased();

    /// \brief Find the gates in the world and store them in this->gates.
    private: bool FindGates();

    /// \brief Find stuff needed for scoring Qual 2
    private: bool FindQual2Stuff();

    /// \brief Find stuff needed for scoring VRC 1
    private: bool FindVRC1Stuff();

    /// \brief Find stuff needed for scoring VRC 3
    private: bool FindVRC3Stuff();

    /// \brief Is the given robot pose "in" the given gate pose?
    /// \param _robotWorldPose Pose of the robot, in the world frame
    /// \param _gateWorldPose Pose of the gate, in the world frame
    /// \param _gateWorldPose Width of the gate
    /// \return If not "in" the gate, return 0; else return -1 if "before" the
    ///         gate, 1 if "after" the gate.
    private: int IsPoseInGate(const gazebo::math::Pose& _robotWorldPose,
                              const gazebo::math::Pose& _gateWorldPose,
                              double _gateWidth);

    /// \brief Data about a gate.
    private: class Gate
             {
               public: Gate(const std::string &_name,
                            unsigned int _number,
                            const gazebo::math::Pose& _pose,
                            double _width)
                         : name(_name), number(_number),
                           pose(_pose), width(_width),
                           passed(false) {}

               /// \brief Less-than operator to allow sorting of a list of
               /// gates by number.
               public: bool operator< (Gate &other)
                       {
                         return (this->number < other.number);
                       }

               /// \brief Name of the gate
               public: std::string name;

               /// \brief Number of the gate
               public: unsigned int number;

               /// \brief Pose of the center of the gate
               public: gazebo::math::Pose pose;

               /// \brief Width of the gate
               public: double width;

               /// \brief Have we passed through this gate yet?
               public: bool passed;
             };

    /// \brief The worlds that we might be scoring; each one can be
    /// slightly different
    private: enum WorldType
             {
               QUAL_1,
               QUAL_2,
               QUAL_3,
               QUAL_4,
               VRC_1,
               VRC_2,
               VRC_3
             };

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Pointer to Atlas.
    private: physics::ModelPtr atlas;

    /// \brief Pointer to drill.
    private: physics::ModelPtr drill;

    /// \brief The bin that will receive the drill
    private: gazebo::math::Box bin;

    /// \brief Pointer to vehicle.
    private: physics::ModelPtr vehicle;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief List of all the gates in the world. We assume that gates have
    /// the names: gate_1, gate_2, ..., gate_n.
    private: std::list<Gate> gates;

    /// \brief Which gate is expected next, expressed as an iterator into
    /// this->gates.
    private: std::list<Gate>::iterator nextGate;

    /// \brief Which side of the next gate we were the last time we checked.
    private: int nextGateSide;

    /// \brief Time at which Atlas passed through the first gate.
    private: gazebo::common::Time startTime;

    /// \brief The completion score, called 'C' in the VRC docs
    private: int completionScore;

    /// \brief How much acceleration must be experienced at the robot's center
    /// of mass to be considering damaging.
    private: double fallAccelThreshold;

    /// \brief How many big falls we've taken
    private: int falls;

    /// \brief Name of the file that we're writing score data to
    private: boost::filesystem::path scoreFilePath;

    /// \brief The stream associated with scoreFilePath
    private: std::ofstream scoreFileStream;

    /// \brief When we last wrote score data to disk
    private: common::Time prevScoreTime;

    /// \brief Last time that we detected a fall
    private: common::Time prevFallTime;

    /// \brief Last time that we calculated acceleration
    private: common::Time prevVelTime;

    /// \brief Velocity at last time we calculated acceleration
    private: gazebo::math::Vector3 prevLinearVel;

    /// \brief Which type of world we're scoring
    private: enum WorldType worldType;
  };
}
#endif
