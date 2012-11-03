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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include "DRCFirehosePlugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCFirehosePlugin::DRCFirehosePlugin()
{
  this->anchorPose = math::Vector3(0, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCFirehosePlugin::~DRCFirehosePlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCFirehosePlugin::Load(physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;
  this->world->EnablePhysicsEngine(true);

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  // Get joints
  for (unsigned int i = 0; i < this->model->GetJointCount(); ++i)
  {
    physics::JointPtr joint = this->model->GetJoint(i);
    this->joints.push_back(joint);
  }

  // Get links
  this->links = this->model->GetAllLinks();

  // Get special coupling links (on the firehose size)
  std::string couplingLinkName = _sdf->GetValueString("coupling_link");
  this->couplingLink = this->model->GetLink(couplingLinkName);
  if (!this->couplingLink)
    gzerr << "coupling [" << couplingLinkName << "] not found\n";

  // Get special links
  std::string spoutModelName = _sdf->GetValueString("spout_model");
  this->spoutModel = this->world->GetModel(spoutModelName);
  std::string spoutLinkName = _sdf->GetValueString("spout_link");
  this->spoutLink = this->spoutModel->GetLink(spoutLinkName);
  if (!this->spoutLink)
    gzerr << "spout [" << spoutLinkName << "] not found\n";

  this->threadPitch = _sdf->GetValueDouble("thread_pitch");

  // this->couplingRelativePose = _sdf->GetValuePose("coupling_relative_pose");
  // this->spoutRelativePose = _sdf->GetValuePose("spout_relative_pose");

  // Reset Time
  this->lastTime = this->world->GetSimTime();

  // Set initial configuration
  this->SetInitialConfiguration();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCFirehosePlugin::UpdateStates, this));
}


////////////////////////////////////////////////////////////////////////////////
void DRCFirehosePlugin::SetInitialConfiguration()
{
  for (unsigned int i = 0; i < this->joints.size(); ++i)
    gzerr << "joint [" << this->joints[i]->GetName() << "]\n";

  for (unsigned int i = 0; i < this->links.size(); ++i)
    gzerr << "link [" << this->links[i]->GetName() << "]\n";

  this->joints[17]->SetAngle(0, -M_PI/4.0);
  this->joints[19]->SetAngle(0, -M_PI/4.0);

  this->Screw(this->couplingLink, this->spoutLink);
}


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCFirehosePlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  if (curTime > this->lastTime)
  {


  }
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void DRCFirehosePlugin::FixLink(physics::LinkPtr _link)
{
  if (!this->fixedJoint)
  {
    this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "revolute",this->model);
    this->fixedJoint->Attach(physics::LinkPtr(), _link);
    // load adds the joint to a vector of shared pointers kept
    // in parent and child links, preventing this from being destroyed.
    this->fixedJoint->Load(physics::LinkPtr(), _link,
      math::Pose(this->anchorPose, math::Quaternion()));
    this->fixedJoint->SetAxis(0, math::Vector3(0, 0, 1));
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);
    this->fixedJoint->SetAnchor(0, this->anchorPose);
    this->fixedJoint->SetName(_link->GetName()+std::string("_world_joint"));
    this->fixedJoint->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void DRCFirehosePlugin::Screw(physics::LinkPtr _link1, physics::LinkPtr _link2)
{
  if (!this->screwJoint)
  {
    this->screwJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "screw",this->model);
    this->screwJoint->Attach(_link1, _link2);
    // load adds the joint to a vector of shared pointers kept
    // in parent and child links, preventing this from being destroyed.
    this->screwJoint->Load(_link1, _link2,
      math::Pose(this->anchorPose, math::Quaternion()));
    this->screwJoint->SetAxis(0, math::Vector3(0, 0, 1));
    this->screwJoint->SetHighStop(0, 0.03);
    this->screwJoint->SetLowStop(0, -0.03);
    this->screwJoint->SetAnchor(0, this->anchorPose);
    // boost::shared_ptr<physics::ScrewJoint< > > sj =
    //   boost::dynamic_pointer_cast<physics::ScrewJoint>(this->screwJoint);
    // sj->SetThreadPitch(0, this->threadPitch);
    // this->screwJoint->SetThreadPitch(0, this->threadPitch);
    this->screwJoint->SetName(_link1->GetName() + std::string("_") +
                              _link2->GetName() + std::string("_joint"));
    this->screwJoint->Init();
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCFirehosePlugin)
}
