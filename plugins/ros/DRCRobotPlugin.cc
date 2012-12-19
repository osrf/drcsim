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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include "DRCRobotPlugin.hh"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCRobotPlugin::DRCRobotPlugin()
{
  this->lFootForce = 0;
  this->lFootTorque = 0;
  this->rFootForce = 0;
  this->rFootTorque = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCRobotPlugin::~DRCRobotPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCRobotPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();
  this->sdf = _sdf;

  // Get joints
  this->rFootJoint = this->model->GetJoint("r_leg_lax");
  if (!this->rFootJoint)
    gzerr << "r_foot_joint not found\n";

  this->lFootJoint = this->model->GetJoint("l_leg_lax");
  if (!this->lFootJoint)
    gzerr << "l_foot_joint not found\n";

  // Get sensors
  this->rFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor("r_foot_contact_sensor"));
  if (!this->rFootContactSensor)
    gzerr << "r_foot_contact_sensor not found\n" << "\n";

  this->lFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor("l_foot_contact_sensor"));
  if (!this->lFootContactSensor)
    gzerr << "l_foot_contact_sensor not found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind( &DRCRobotPlugin::LoadThread,this ) );
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCRobotPlugin::LoadThread()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",
      ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("");

  // ros publication / subscription
  this->pub_status_ =
    this->rosnode_->advertise<std_msgs::String>("drc_robot/status", 10);

  this->pub_l_foot_ft_ =
    this->rosnode_->advertise<geometry_msgs::Wrench>(
      "drc_robot/l_foot_ft", 10);

  this->pub_r_foot_ft_ =
    this->rosnode_->advertise<geometry_msgs::Wrench>(
      "drc_robot/r_foot_ft", 10);

  this->pub_l_foot_contact_ =
    this->rosnode_->advertise<geometry_msgs::Wrench>(
      "drc_robot/l_foot_contact", 10);

  this->pub_r_foot_contact_ =
    this->rosnode_->advertise<geometry_msgs::Wrench>(
      "drc_robot/r_foot_contact", 10);

  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &DRCRobotPlugin::QueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&DRCRobotPlugin::UpdateStates, this));

  this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
     boost::bind(&DRCRobotPlugin::OnLContactUpdate, this));

  this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
     boost::bind(&DRCRobotPlugin::OnRContactUpdate, this));
}

void DRCRobotPlugin::UpdateStates()
{
  /// @todo:  robot internals
  /// self diagnostics, damages, etc.

  if (this->pub_status_.getNumSubscribers() > 0)
  {
    double cur_time = this->world->GetSimTime().Double();

    if (cur_time - this->lastUpdateTime >= 1.0/this->updateRate)
    {
      this->lastUpdateTime = cur_time;
      std_msgs::String msg;
      msg.data = "ok";
      this->pub_status_.publish(msg);
    }
  }

  // this->rFootContactSensor->GetContact
  {
    physics::JointWrench wrench = this->rFootJoint->GetForceTorque(0);
    geometry_msgs::Wrench msg;
    msg.force.x = wrench.body1Force.x;
    msg.force.y = wrench.body1Force.y;
    msg.force.z = wrench.body1Force.z;
    msg.torque.x = wrench.body1Torque.x;
    msg.torque.y = wrench.body1Torque.y;
    msg.torque.z = wrench.body1Torque.z;
    this->pub_r_foot_ft_.publish(msg);
  }

  {
    physics::JointWrench wrench = this->lFootJoint->GetForceTorque(0);
    geometry_msgs::Wrench msg;
    msg.force.x = wrench.body1Force.x;
    msg.force.y = wrench.body1Force.y;
    msg.force.z = wrench.body1Force.z;
    msg.torque.x = wrench.body1Torque.x;
    msg.torque.y = wrench.body1Torque.y;
    msg.torque.z = wrench.body1Torque.z;
    this->pub_l_foot_ft_.publish(msg);
  }
}

void DRCRobotPlugin::OnLContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->lFootContactSensor->GetContacts();


  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    // gzerr << "Collision between[" << contacts.contact(i).collision1()
    //           << "] and [" << contacts.contact(i).collision2() << "]\n";
    // gzerr << " t[" << this->world->GetSimTime()
    //       << "] i[" << i
    //       << "] s[" << contacts.contact(i).time().sec()
    //       << "] n[" << contacts.contact(i).time().nsec()
    //       << "] size[" << contacts.contact(i).position_size()
    //       << "]\n";

    // common::Time contactTime(contacts.contact(i).time().sec(),
    //                          contacts.contact(i).time().nsec());
    math::Vector3 fTotal;
    math::Vector3 tTotal;
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      // gzerr << j << "  Position:"
      //       << contacts.contact(i).position(j).x() << " "
      //       << contacts.contact(i).position(j).y() << " "
      //       << contacts.contact(i).position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contacts.contact(i).normal(j).x() << " "
      //       << contacts.contact(i).normal(j).y() << " "
      //       << contacts.contact(i).normal(j).z() << "\n";
      // gzerr << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_force().x(),
                            contacts.contact(i).wrench(j).body_1_force().y(),
                            contacts.contact(i).wrench(j).body_1_force().z());
      tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_torque().x(),
                            contacts.contact(i).wrench(j).body_1_torque().y(),
                            contacts.contact(i).wrench(j).body_1_torque().z());
    }
    // low pass filter over time
    double e = 0.99;
    this->lFootForce = e * this->lFootForce + (1.0 - e) * fTotal;
    this->lFootTorque = e * this->lFootTorque + (1.0 - e) * tTotal;

    geometry_msgs::Wrench msg;
    msg.force.x = this->lFootForce.x;
    msg.force.y = this->lFootForce.y;
    msg.force.z = this->lFootForce.z;
    msg.torque.x = this->lFootTorque.x;
    msg.torque.y = this->lFootTorque.y;
    msg.torque.z = this->lFootTorque.z;
    this->pub_l_foot_contact_.publish(msg);
  }
}

void DRCRobotPlugin::OnRContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->rFootContactSensor->GetContacts();


  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    // gzerr << "Collision between[" << contacts.contact(i).collision1()
    //           << "] and [" << contacts.contact(i).collision2() << "]\n";
    // gzerr << " t[" << this->world->GetSimTime()
    //       << "] i[" << i
    //       << "] s[" << contacts.contact(i).time().sec()
    //       << "] n[" << contacts.contact(i).time().nsec()
    //       << "] size[" << contacts.contact(i).position_size()
    //       << "]\n";

    // common::Time contactTime(contacts.contact(i).time().sec(),
    //                          contacts.contact(i).time().nsec());
    math::Vector3 fTotal;
    math::Vector3 tTotal;
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      // gzerr << j << "  Position:"
      //       << contacts.contact(i).position(j).x() << " "
      //       << contacts.contact(i).position(j).y() << " "
      //       << contacts.contact(i).position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contacts.contact(i).normal(j).x() << " "
      //       << contacts.contact(i).normal(j).y() << " "
      //       << contacts.contact(i).normal(j).z() << "\n";
      // gzerr << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_force().x(),
                            contacts.contact(i).wrench(j).body_1_force().y(),
                            contacts.contact(i).wrench(j).body_1_force().z());
      tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_torque().x(),
                            contacts.contact(i).wrench(j).body_1_torque().y(),
                            contacts.contact(i).wrench(j).body_1_torque().z());
    }
    // low pass filter over time
    double e = 0.99;
    this->rFootForce = e * this->rFootForce + (1.0 - e) * fTotal;
    this->rFootTorque = e * this->rFootTorque + (1.0 - e) * tTotal;

    geometry_msgs::Wrench msg;
    msg.force.x = this->rFootForce.x;
    msg.force.y = this->rFootForce.y;
    msg.force.z = this->rFootForce.z;
    msg.torque.x = this->rFootTorque.x;
    msg.torque.y = this->rFootTorque.y;
    msg.torque.z = this->rFootTorque.z;
    this->pub_r_foot_contact_.publish(msg);
  }
}

void DRCRobotPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(DRCRobotPlugin)
}
