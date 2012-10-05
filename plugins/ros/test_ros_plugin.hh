/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig
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
 * Desc: Test plugin for compiling against ROS
 * Author: John Hsu
 * Date: 1 Oct 2012
 * SVN info: $Id$
 */
#ifndef GAZEBO_TEST_ROS_PLUGIN_HH
#define GAZEBO_TEST_ROS_PLUGIN_HH

#include <boost/thread.hpp>

#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include "boost/thread/mutex.hpp"

namespace gazebo
{
  class TestROSPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TestROSPlugin();

    /// \brief Destructor
    public: virtual ~TestROSPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
  };
}
#endif
