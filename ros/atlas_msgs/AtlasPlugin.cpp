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
 * Desc: Plugin for the Atlas robot
 * Author: John Hsu
 * Date: December 2012
 */
#include "AtlasPlugin.h"

#include "sensor_msgs/Imu.h"
#include "atlas_msgs/ForceTorqueSensors.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
AtlasPlugin::AtlasPlugin()
{
  this->lFootForce = 0;
  this->lFootTorque = 0;
  this->rFootForce = 0;
  this->rFootTorque = 0;
  this->imuLinkName = "imu_link";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AtlasPlugin::~AtlasPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AtlasPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();
  this->sdf = _sdf;
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // initialize imu
  this->lastImuTime = this->world->GetSimTime();

  // get joints
  this->joints.push_back(model->GetJoint("back_lbz")); 
  this->joints.push_back(model->GetJoint("back_mby")); 
  this->joints.push_back(model->GetJoint("back_ubx")); 
  this->joints.push_back(model->GetJoint("neck_ay")); 
  this->joints.push_back(model->GetJoint("l_leg_uhz")); 
  this->joints.push_back(model->GetJoint("l_leg_mhx")); 
  this->joints.push_back(model->GetJoint("l_leg_lhy")); 
  this->joints.push_back(model->GetJoint("l_leg_kny")); 
  this->joints.push_back(model->GetJoint("l_leg_uay")); 
  this->joints.push_back(model->GetJoint("l_leg_lax")); 
  this->joints.push_back(model->GetJoint("r_leg_uhz")); 
  this->joints.push_back(model->GetJoint("r_leg_mhx")); 
  this->joints.push_back(model->GetJoint("r_leg_lhy")); 
  this->joints.push_back(model->GetJoint("r_leg_kny")); 
  this->joints.push_back(model->GetJoint("r_leg_uay")); 
  this->joints.push_back(model->GetJoint("r_leg_lax")); 
  this->joints.push_back(model->GetJoint("l_arm_usy")); 
  this->joints.push_back(model->GetJoint("l_arm_shx")); 
  this->joints.push_back(model->GetJoint("l_arm_ely")); 
  this->joints.push_back(model->GetJoint("l_arm_elx")); 
  this->joints.push_back(model->GetJoint("l_arm_uwy")); 
  this->joints.push_back(model->GetJoint("l_arm_mwx")); 
  this->joints.push_back(model->GetJoint("r_arm_usy")); 
  this->joints.push_back(model->GetJoint("r_arm_shx")); 
  this->joints.push_back(model->GetJoint("r_arm_ely")); 
  this->joints.push_back(model->GetJoint("r_arm_elx")); 
  this->joints.push_back(model->GetJoint("r_arm_uwy")); 
  this->joints.push_back(model->GetJoint("r_arm_mwx")); 

  // Get imu link
  this->imuLink = this->model->GetLink(this->imuLinkName);
  if (!this->imuLink)
    gzerr << this->imuLinkName << " not found\n";

  // initialize imu reference pose
  this->imuReferencePose = this->imuLink->GetWorldPose();
  this->imuLastLinearVel = imuReferencePose.rot.RotateVector(
    this->imuLink->GetWorldLinearVel());
  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)

  // Get force torque joints
  this->lWristJoint = this->model->GetJoint("l_arm_mwx");
  if (!this->lWristJoint)
    gzerr << "left wrist joint (l_arm_mwx) not found\n";

  this->rWristJoint = this->model->GetJoint("r_arm_mwx");
  if (!this->rWristJoint)
    gzerr << "right wrist joint (r_arm_mxw) not found\n";

  this->rAnkleJoint = this->model->GetJoint("r_leg_lax");
  if (!this->rAnkleJoint)
    gzerr << "right ankle joint (r_leg_lax) not found\n";

  this->lAnkleJoint = this->model->GetJoint("l_leg_lax");
  if (!this->lAnkleJoint)
    gzerr << "left ankle joint (l_leg_lax) not found\n";

  // Get sensors
  this->rFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::r_foot::"
        "r_foot_contact_sensor"));
  if (!this->rFootContactSensor)
    gzerr << "r_foot_contact_sensor not found\n" << "\n";

  this->lFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::l_foot::"
        "l_foot_contact_sensor"));
  if (!this->lFootContactSensor)
    gzerr << "l_foot_contact_sensor not found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&AtlasPlugin::DeferredLoad,this ));
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AtlasPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // ros publication / subscription
  this->pubStatus =
    this->rosNode->advertise<std_msgs::String>("atlas/status", 10);

  this->pubLAnkleFT =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/l_ankle_ft", 10);

  this->pubRAnkleFT =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/r_ankle_ft", 10);

  this->pubLWristFT =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/l_wrist_ft", 10);

  this->pubRWristFT =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/r_wrist_ft", 10);

  this->pubLFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/l_foot_contact", 10);

  this->pubRFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/r_foot_contact", 10);

  // publish imu data
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "atlas/imu", 10);

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind( &AtlasPlugin::RosQueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&AtlasPlugin::UpdateStates, this));

  this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnLContactUpdate, this));

  this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnRContactUpdate, this));
}

void AtlasPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();
  /// @todo:  robot internals
  /// self diagnostics, damages, etc.

  if (this->pubStatus.getNumSubscribers() > 0)
  {
    double cur_time = this->world->GetSimTime().Double();

    if (cur_time - this->lastStatusTime >= 1.0/this->updateRate)
    {
      this->lastStatusTime = cur_time;
      std_msgs::String msg;
      msg.data = "ok";
      this->pubStatus.publish(msg);
    }
  }

  if (curTime > this->lastControllerUpdateTime)
  {
    // get imu data from imu link
    if (this->imuLink && curTime > this->lastImuTime)
    {
      double dt = (curTime - this->lastImuTime).Double();

      // Get imuLnk Pose/Orientation
      math::Pose imuPose = this->imuLink->GetWorldPose();
      math::Vector3 imuLinearVel = imuPose.rot.RotateVector(
        this->imuLink->GetWorldLinearVel());

      sensor_msgs::Imu msg;
      msg.header.frame_id = this->imuLinkName;
      msg.header.stamp = ros::Time(curTime.Double());

      // compute angular rates
      {
        // get world twist and convert to local frame
        math::Vector3 wLocal = imuPose.rot.RotateVector(
          this->imuLink->GetWorldAngularVel());
        msg.angular_velocity.x = wLocal.x;
        msg.angular_velocity.y = wLocal.y;
        msg.angular_velocity.z = wLocal.z;
      }

      // compute acceleration
      {
        math::Vector3 accel = imuLinearVel - this->imuLastLinearVel;
        double imuDdx = accel.x;
        double imuDdy = accel.y;
        double imuDdz = accel.z;

        msg.linear_acceleration.x = imuDdx;
        msg.linear_acceleration.y = imuDdy;
        msg.linear_acceleration.z = imuDdz;

        this->imuLastLinearVel = imuLinearVel;
      }

      // compute orientation
      {
        // Get IMU rotation relative to Initial IMU Reference Pose
        math::Quaternion imuRot =
          imuPose.rot * this->imuReferencePose.rot.GetInverse();

        double imuOrientationEstimate[4];
        imuOrientationEstimate[0] = imuRot.w;
        imuOrientationEstimate[1] = imuRot.x;
        imuOrientationEstimate[2] = imuRot.y;
        imuOrientationEstimate[3] = imuRot.z;

        msg.orientation.x = imuRot.x;
        msg.orientation.y = imuRot.y;
        msg.orientation.z = imuRot.z;
        msg.orientation.w = imuRot.w;
      }

      this->pubImu.publish(msg);

      // update time
      this->lastImuTime = curTime.Double();
    }

/* Not yet available in gazebo 1.3
    // get force torque at left ankle and publish
    if (this->lAnkleJoint)
    {
      physics::JointWrench wrench = this->lAnkleJoint->GetForceTorque(0);
      geometry_msgs::Wrench msg;
      msg.force.x = wrench.body1Force.x;
      msg.force.y = wrench.body1Force.y;
      msg.force.z = wrench.body1Force.z;
      msg.torque.x = wrench.body1Torque.x;
      msg.torque.y = wrench.body1Torque.y;
      msg.torque.z = wrench.body1Torque.z;
      this->pubLAnkleFT.publish(msg);

      double l_foot_sensors_fz = wrench.body1Force.z;
      double l_foot_sensors_mx = wrench.body1Torque.x;
      double l_foot_sensors_my = wrench.body1Torque.y;
    }

    // get force torque at right ankle and publish
    if (this->rAnkleJoint)
    {
      physics::JointWrench wrench = this->rAnkleJoint->GetForceTorque(0);
      geometry_msgs::Wrench msg;
      msg.force.x = wrench.body1Force.x;
      msg.force.y = wrench.body1Force.y;
      msg.force.z = wrench.body1Force.z;
      msg.torque.x = wrench.body1Torque.x;
      msg.torque.y = wrench.body1Torque.y;
      msg.torque.z = wrench.body1Torque.z;
      this->pubRAnkleFT.publish(msg);

      double r_foot_sensors_fz = wrench.body1Force.z;
      double r_foot_sensors_mx = wrench.body1Torque.x;
      double r_foot_sensors_my = wrench.body1Torque.y;
    }

    // get force torque at left wrist and publish
    if (this->lWristJoint)
    {
      physics::JointWrench wrench = this->lWristJoint->GetForceTorque(0);
      geometry_msgs::Wrench msg;
      msg.force.x = wrench.body1Force.x;
      msg.force.y = wrench.body1Force.y;
      msg.force.z = wrench.body1Force.z;
      msg.torque.x = wrench.body1Torque.x;
      msg.torque.y = wrench.body1Torque.y;
      msg.torque.z = wrench.body1Torque.z;
      this->pubLWristFT.publish(msg);

      double l_wrist_sensors_fx = wrench.body1Force.x;
      double l_wrist_sensors_fy = wrench.body1Force.y;
      double l_wrist_sensors_fz = wrench.body1Force.z;
      double l_wrist_sensors_mx = wrench.body1Torque.x;
      double l_wrist_sensors_my = wrench.body1Torque.y;
      double l_wrist_sensors_mz = wrench.body1Torque.z;
    }

    // get force torque at right wrist and publish
    if (this->rWristJoint)
    {
      physics::JointWrench wrench = this->rWristJoint->GetForceTorque(0);
      geometry_msgs::Wrench msg;
      msg.force.x = wrench.body1Force.x;
      msg.force.y = wrench.body1Force.y;
      msg.force.z = wrench.body1Force.z;
      msg.torque.x = wrench.body1Torque.x;
      msg.torque.y = wrench.body1Torque.y;
      msg.torque.z = wrench.body1Torque.z;
      this->pubRWristFT.publish(msg);

      double r_wrist_sensors_fx = wrench.body1Force.x;
      double r_wrist_sensors_fy = wrench.body1Force.y;
      double r_wrist_sensors_fz = wrench.body1Force.z;
      double r_wrist_sensors_mx = wrench.body1Torque.x;
      double r_wrist_sensors_my = wrench.body1Torque.y;
      double r_wrist_sensors_mz = wrench.body1Torque.z;
    }
*/

    this->lastControllerUpdateTime = curTime;
  }

}

void AtlasPlugin::OnLContactUpdate()
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
    this->lFootForce = this->lFootForce * e + fTotal * (1.0 - e);
    this->lFootTorque = this->lFootTorque * e + tTotal * (1.0 - e);

    geometry_msgs::Wrench msg;
    msg.force.x = this->lFootForce.x;
    msg.force.y = this->lFootForce.y;
    msg.force.z = this->lFootForce.z;
    msg.torque.x = this->lFootTorque.x;
    msg.torque.y = this->lFootTorque.y;
    msg.torque.z = this->lFootTorque.z;
    this->pubLFootContact.publish(msg);
  }
}

void AtlasPlugin::OnRContactUpdate()
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
    this->rFootForce = this->rFootForce * e + fTotal * (1.0 - e);
    this->rFootTorque = this->rFootTorque * e + tTotal * (1.0 - e);

    geometry_msgs::Wrench msg;
    msg.force.x = this->rFootForce.x;
    msg.force.y = this->rFootForce.y;
    msg.force.z = this->rFootForce.z;
    msg.torque.x = this->rFootTorque.x;
    msg.torque.y = this->rFootTorque.y;
    msg.torque.z = this->rFootTorque.z;
    this->pubRFootContact.publish(msg);
  }
}

void AtlasPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)
}
