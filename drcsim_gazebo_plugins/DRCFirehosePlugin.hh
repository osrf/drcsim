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
#ifndef GAZEBO_DRC_FIREHOSE_PLUGIN_HH
#define GAZEBO_DRC_FIREHOSE_PLUGIN_HH

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
  /// \addtogroup drc_plugin
  /// \{
  class DRCFirehosePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DRCFirehosePlugin();

    /// \brief Destructor
    public: virtual ~DRCFirehosePlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief pin a link to the world
    private: void FixLink(physics::LinkPtr _link);

    /// \brief add a constraint between 2 links
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief Set configuration of the hose
    private: void SetInitialConfiguration();

    /// Continuously checks DRC firehose coupling pose against spigot pose.
    /// If sufficient alignment between the two exists,
    /// and the relative motion of the two allows for thread initiation,
    /// dynamically create a screw joint constraint between the objects.
    private: bool CheckThreadStart();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// joint for pinning a link to the world
    private: physics::JointPtr fixedJoint;

    /// screw joint
    private: physics::JointPtr screwJoint;
    private: double threadPitch;

    /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::Joint_V joints;
    private: physics::Link_V links;
    private: common::Time lastTime;

    private: physics::LinkPtr couplingLink;
    private: physics::ModelPtr spoutModel;
    private: physics::LinkPtr spoutLink;
    private: math::Pose couplingRelativePose;
  };
/// \} 
}
#endif
