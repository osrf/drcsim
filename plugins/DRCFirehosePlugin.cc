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
#include "gazebo/physics/ScrewJoint.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCFirehosePlugin::DRCFirehosePlugin()
{
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
  this->joints = this->model->GetJoints();

  // Get links
  this->links = this->model->GetLinks();

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

  this->couplingRelativePose = _sdf->GetValuePose("coupling_relative_pose");

  // Reset Time
  this->lastTime = this->world->GetSimTime();

  // Set initial configuration
  // this->SetInitialConfiguration();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&DRCFirehosePlugin::UpdateStates, this));
}


////////////////////////////////////////////////////////////////////////////////
void DRCFirehosePlugin::SetInitialConfiguration()
{
  // for (unsigned int i = 0; i < this->joints.size(); ++i)
  //   gzerr << "joint [" << this->joints[i]->GetName() << "]\n";

  // for (unsigned int i = 0; i < this->links.size(); ++i)
  //   gzerr << "link [" << this->links[i]->GetName() << "]\n";

  this->joints[17]->SetAngle(0, -M_PI/4.0);
  this->joints[19]->SetAngle(0, -M_PI/4.0);
}


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCFirehosePlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  if (curTime > this->lastTime)
  {
    this->CheckThreadStart();
  }
}

bool DRCFirehosePlugin::CheckThreadStart()
{
  // gzerr << "coupling [" << this->couplingLink->GetWorldPose() << "]\n";
  // gzerr << "spout [" << this->spoutLink->GetWorldPose() << "]\n"
  math::Pose connectPose(this->couplingRelativePose);
  math::Pose relativePose = this->couplingLink->GetWorldPose() -
                            this->spoutLink->GetWorldPose();

  math::Pose connectOffset = relativePose - connectPose;

  double posErr = (relativePose.pos - connectPose.pos).GetLength();
  double rotErr = (relativePose.rot.GetZAxis() -
                   connectPose.rot.GetZAxis()).GetLength();

  // gzdbg << "connect offset [" << connectOffset
  //       << "] xyz [" << posErr
  //       << "] rpy [" << rotErr
  //       << "]\n";

  if (!this->screwJoint)
  {
    if (posErr < 0.01 && rotErr < 0.01)
    {
      this->screwJoint = this->AddJoint(this->world, this->model,
                                        this->spoutLink, this->couplingLink,
                                        "screw",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        20.0/1000, -0.5/1000);
                                        // 20.0, -0.5); // recover threadPitch
    }
  }
  else
  {
    // check joint position to disconnect
    double position = this->screwJoint->GetAngle(0).Radian();
    // gzerr << "position " << position << "\n";
    if (position < -0.0003)
      this->RemoveJoint(this->screwJoint);
  }
  return true;
}


////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
physics::JointPtr DRCFirehosePlugin::AddJoint(physics::WorldPtr _world,
                                              physics::ModelPtr _model,
                                              physics::LinkPtr _link1,
                                              physics::LinkPtr _link2,
                                              std::string _type,
                                              math::Vector3 _anchor,
                                              math::Vector3 _axis,
                                              double _upper, double _lower)
{
  physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
    _type, _model);
  joint->Attach(_link1, _link2);
  // load adds the joint to a vector of shared pointers kept
  // in parent and child links, preventing joint from being destroyed.
  joint->Load(_link1, _link2, _anchor);
  // joint->SetAnchor(0, _anchor);
  joint->SetAxis(0, _axis);
  joint->SetHighStop(0, _upper);
  joint->SetLowStop(0, _lower);

  /// threadPitch a function parameter, but only works with default branch
  /* awaiting gazebo 1.3 in the future
  joint->SetAttribute("thread_pitch", 0, this->threadPitch);
  */

  /* SetThreadPitch should not be exposed in generic Joint
  joint->SetThreadPitch(0, this->threadPitch);
  */

  /* ODEJoint class is not exposed
  physics::ScrewJoint<physics::ODEJoint>* screwJoint =
    dynamic_cast<physics::ScrewJoint<physics::ODEJoint>* >(this);
  if (screwJoint != NULL)
    screwJoint->SetThreadPitch(0, this->threadPitch);
  */

  /* trying through sdf, but sdf is empty
  sdf::ElementPtr sdf = joint->GetSDF();
  sdf->PrintValues("");
  sdf->GetElement("thread_pitch")->Set(this->threadPitch);
  joint->UpdateParameters(sdf);
  joint->Load(sdf);
  */

  joint->SetName(_link1->GetName() + std::string("_") +
                            _link2->GetName() + std::string("_joint"));
  joint->Init();

  // disable collision between the link pair
  _link1->SetCollideMode("fixed");
  _link2->SetCollideMode("fixed");
  return joint;
}

////////////////////////////////////////////////////////////////////////////////
// remove a joint
void DRCFirehosePlugin::RemoveJoint(physics::JointPtr &_joint)
{
  if (_joint)
  {
    // reenable collision between the link pair
    physics::LinkPtr parent = _joint->GetParent();
    physics::LinkPtr child = _joint->GetChild();
    parent->SetCollideMode("all");
    child->SetCollideMode("all");

    _joint->Detach();
    _joint.reset();
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCFirehosePlugin)
}
