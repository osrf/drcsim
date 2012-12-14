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
 * Desc: Test plugin for compiling against ROS
 * Author: John Hsu
 * Date: 1 Oct 2012
 * SVN info: $Id$
 */

#include "test_ros_plugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
TestROSPlugin::TestROSPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TestROSPlugin::~TestROSPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void TestROSPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Parse _sdf to get parameters for this plugin if you like

  // Get ponters to world and model
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
     boost::bind(&TestROSPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void TestROSPlugin::UpdateStates()
{
}

GZ_REGISTER_MODEL_PLUGIN(TestROSPlugin)
}
