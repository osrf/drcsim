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
#ifndef GAZEBO_DRC_FIREHOSE_PLUGIN_HH
#define GAZEBO_DRC_FIREHOSE_PLUGIN_HH

#include <boost/thread.hpp>

#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include "boost/thread/mutex.hpp"

namespace gazebo
{
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
    private: void RemoveJoint(physics::JointPtr _joint);

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
    private: math::Pose spoutRelativePose;

  };
/** \} */
/// @}
}
#endif
