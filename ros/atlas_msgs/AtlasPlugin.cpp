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

#include <string>

#include "AtlasPlugin.h"

#include "sensor_msgs/Imu.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

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
  this->pubQueeuThread.join();
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
  this->jointNames.push_back("back_lbz");
  this->jointNames.push_back("back_mby");
  this->jointNames.push_back("back_ubx");
  this->jointNames.push_back("neck_ay");
  this->jointNames.push_back("l_leg_uhz");
  this->jointNames.push_back("l_leg_mhx");
  this->jointNames.push_back("l_leg_lhy");
  this->jointNames.push_back("l_leg_kny");
  this->jointNames.push_back("l_leg_uay");
  this->jointNames.push_back("l_leg_lax");
  this->jointNames.push_back("r_leg_uhz");
  this->jointNames.push_back("r_leg_mhx");
  this->jointNames.push_back("r_leg_lhy");
  this->jointNames.push_back("r_leg_kny");
  this->jointNames.push_back("r_leg_uay");
  this->jointNames.push_back("r_leg_lax");
  this->jointNames.push_back("l_arm_usy");
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back("l_arm_uwy");
  this->jointNames.push_back("l_arm_mwx");
  this->jointNames.push_back("r_arm_usy");
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back("r_arm_uwy");
  this->jointNames.push_back("r_arm_mwx");

  this->joints.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])
    {
      ROS_ERROR("atlas robot expected joint[%s] not present, plugin not loaded",
        this->jointNames[i].c_str());
      return;
    }
  }

  this->errorTerms.resize(this->joints.size());

  this->jointStates.name.resize(this->joints.size());
  this->jointStates.position.resize(this->joints.size());
  this->jointStates.velocity.resize(this->joints.size());
  this->jointStates.effort.resize(this->joints.size());

  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->jointStates.name[i] = this->jointNames[i];

  this->jointCommands.name.resize(this->joints.size());
  this->jointCommands.position.resize(this->joints.size());
  this->jointCommands.velocity.resize(this->joints.size());
  this->jointCommands.effort.resize(this->joints.size());
  this->jointCommands.kp_position.resize(this->joints.size());
  this->jointCommands.ki_position.resize(this->joints.size());
  this->jointCommands.kd_position.resize(this->joints.size());
  this->jointCommands.kp_velocity.resize(this->joints.size());
  this->jointCommands.i_effort_min.resize(this->joints.size());
  this->jointCommands.i_effort_max.resize(this->joints.size());

  for (unsigned i = 0; i < this->joints.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i = 0;
    this->errorTerms[i].qd_p = 0;
    this->jointCommands.name[i] = this->joints[i]->GetScopedName();
    this->jointCommands.position[i] = 0;
    this->jointCommands.velocity[i] = 0;
    this->jointCommands.effort[i] = 0;
    this->jointCommands.kp_position[i] = 0;
    this->jointCommands.ki_position[i] = 0;
    this->jointCommands.kd_position[i] = 0;
    this->jointCommands.kp_velocity[i] = 0;
    this->jointCommands.i_effort_min[i] = 0;
    this->jointCommands.i_effort_max[i] = 0;
  }

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
    boost::bind(&AtlasPlugin::DeferredLoad, this));
}


////////////////////////////////////////////////////////////////////////////////
// Set Joint Commands
void AtlasPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg)
{
  struct timespec tv;
  clock_gettime(0, &tv);
  gazebo::common::Time gtv = tv;

  static common::Time last;

  // round trip, JS published by AtlasPlugin, received by pub_joint_command
  // and republished over JC, received by AtlasPlugin
  if (this->world->GetSimTime() > last)
  ROS_ERROR("now [%f] js pub sim time [%f] receive sim time [%f] diff [%f]",
    gtv.Double(),
    _msg->header.stamp.toSec(), this->world->GetSimTime().Double(),
    (last - this->world->GetSimTime()).Double());
    //this->world->GetSimTime().Double() - _msg->header.stamp.toSec());

  last = this->world->GetSimTime();
    
  if (_msg->name.size() == this->jointCommands.name.size() &&
      _msg->position.size() == this->jointCommands.position.size() &&
      _msg->velocity.size() == this->jointCommands.velocity.size() &&
      _msg->effort.size() == this->jointCommands.effort.size() &&
      _msg->kp_position.size() == this->jointCommands.kp_position.size() &&
      _msg->ki_position.size() == this->jointCommands.ki_position.size() &&
      _msg->kd_position.size() == this->jointCommands.kd_position.size() &&
      _msg->kp_velocity.size() == this->jointCommands.kp_velocity.size() &&
      _msg->i_effort_min.size() == this->jointCommands.i_effort_min.size() &&
      _msg->i_effort_max.size() == this->jointCommands.i_effort_max.size())
  {
    /// \todo: make this smarter and skip messages if not specified
    for (unsigned i = 0; i < this->joints.size(); ++i)
    {
      // this->jointCommands.name[i] = this->joints[i]->GetScopedName();
      this->jointCommands.position[i] = _msg->position[i];
      this->jointCommands.velocity[i] = _msg->velocity[i];
      this->jointCommands.effort[i] = _msg->effort[i];
      this->jointCommands.kp_position[i] = _msg->kp_position[i];
      this->jointCommands.ki_position[i] = _msg->ki_position[i];
      this->jointCommands.kd_position[i] = _msg->kd_position[i];
      this->jointCommands.kp_velocity[i] = _msg->kp_velocity[i];
      this->jointCommands.i_effort_min[i] = _msg->i_effort_min[i];
      this->jointCommands.i_effort_max[i] = _msg->i_effort_max[i];
    }
  }
  else
  {
    ROS_DEBUG("joint commands message contains different number of joints"
              " than expected");
  }
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

  // pull down controller parameters
  for (unsigned int joint = 0; joint < this->joints.size(); ++joint)
  {
    char joint_ns[200] = "";
    snprintf(joint_ns, sizeof(joint_ns), "atlas_controller/gains/%s/",
             this->joints[joint]->GetName().c_str());
    // this is so ugly
    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
    string p_str = string(joint_ns)+"p";
    string i_str = string(joint_ns)+"i";
    string d_str = string(joint_ns)+"d";
    string i_clamp_str = string(joint_ns)+"i_clamp";
    if (!this->rosNode->getParam(p_str, p_val) ||
        !this->rosNode->getParam(i_str, i_val) ||
        !this->rosNode->getParam(d_str, d_val) ||
        !this->rosNode->getParam(i_clamp_str, i_clamp_val))
    {
      ROS_ERROR("couldn't find a param for %s", joint_ns);
      continue;
    }
    this->jointCommands.kp_position[joint]  =  p_val;
    this->jointCommands.ki_position[joint]  =  i_val;
    this->jointCommands.kd_position[joint]  =  d_val;
    this->jointCommands.i_effort_min[joint] = -i_clamp_val;
    this->jointCommands.i_effort_max[joint] =  i_clamp_val;
  }

  // ROS Controller API
  /// brief broadcasts the robot states
  ros::AdvertiseOptions pub_joint_states_ao =
    ros::AdvertiseOptions::create<sensor_msgs::JointState>(
      "atlas/joint_states", 10,
      boost::bind(&AtlasPlugin::foo, this),
      boost::bind(&AtlasPlugin::foo, this),
      ros::VoidPtr(), &this->rosPubQueue);
  this->pubJointStates = this->rosNode->advertise(pub_joint_states_ao);
  // this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
  //   "atlas/joint_states", 10);

  this->pubForceTorqueSensors =
    this->rosNode->advertise<atlas_msgs::ForceTorqueSensors>(
    "atlas/force_torque_sensors", 10);

  // ros publication / subscription
  this->pubStatus =
    this->rosNode->advertise<std_msgs::String>("atlas/status", 10);

  this->pubLFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/l_foot_contact", 10);

  this->pubRFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/r_foot_contact", 10);

  // ros topic subscribtions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "atlas/joint_commands", 10,
    boost::bind(&AtlasPlugin::SetJointCommands, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands=
    this->rosNode->subscribe(jointCommandsSo);

  // publish imu data
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>("atlas/imu", 10);

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&AtlasPlugin::RosQueueThread, this));

  this->pubQueeuThread = boost::thread(
    boost::bind(&AtlasPlugin::RosPubQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&AtlasPlugin::UpdateStates, this));

  // on contact
  this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnLContactUpdate, this));

  this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnRContactUpdate, this));
}

void AtlasPlugin::UpdateStates()
{
  common::Timer timer;
  common::Timer timer1;
  timer.Start();
  timer1.Start();

  common::Time curTime = this->world->GetSimTime();

  static gazebo::common::Time last_gtv;
  struct timespec tv;
  clock_gettime(0, &tv);
  gazebo::common::Time gtv = tv;
  gzerr << "cur sim Time[" << curTime.Double()*1000.0
        << "] dt[" << (curTime - lastControllerUpdateTime).Double()*1000.0
        << "] rt[" << gtv.Double()*1000.0
        << "] drt[" << (gtv - last_gtv).Double()*1000.0
        << "]\n";
  last_gtv = gtv;

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
      // Get imuLnk Pose/Orientation
      math::Pose imuPose = this->imuLink->GetWorldPose();
      math::Vector3 imuLinearVel = imuPose.rot.RotateVector(
        this->imuLink->GetWorldLinearVel());

      sensor_msgs::Imu imuMsg;
      imuMsg.header.frame_id = this->imuLinkName;
      imuMsg.header.stamp = ros::Time(curTime.Double());

      // compute angular rates
      {
        // get world twist and convert to local frame
        math::Vector3 wLocal = imuPose.rot.RotateVector(
          this->imuLink->GetWorldAngularVel());
        imuMsg.angular_velocity.x = wLocal.x;
        imuMsg.angular_velocity.y = wLocal.y;
        imuMsg.angular_velocity.z = wLocal.z;
      }

      // compute acceleration
      {
        math::Vector3 accel = imuLinearVel - this->imuLastLinearVel;
        double imuDdx = accel.x;
        double imuDdy = accel.y;
        double imuDdz = accel.z;

        imuMsg.linear_acceleration.x = imuDdx;
        imuMsg.linear_acceleration.y = imuDdy;
        imuMsg.linear_acceleration.z = imuDdz;

        this->imuLastLinearVel = imuLinearVel;
      }

      // compute orientation
      {
        // Get IMU rotation relative to Initial IMU Reference Pose
        math::Quaternion imuRot =
          imuPose.rot * this->imuReferencePose.rot.GetInverse();

        imuMsg.orientation.x = imuRot.x;
        imuMsg.orientation.y = imuRot.y;
        imuMsg.orientation.z = imuRot.z;
        imuMsg.orientation.w = imuRot.w;
      }

      this->pubImu.publish(imuMsg);

      // update time
      this->lastImuTime = curTime.Double();
      gzerr << "  AP imu [" << timer.GetElapsed().Double()*1000.0 << "]\n";
      timer.Start();
    }

#if GAZEBO_MINOR_VERSION > 3
    this->forceTorqueSensorsMsg.header.stamp =
      ros::Time(curTime.sec, curTime.nsec);

    // get force torque at left ankle and publish
    if (this->lAnkleJoint)
    {
      physics::JointWrench wrench = this->lAnkleJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.l_foot.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.l_foot.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.l_foot.torque.y = wrench.body1Torque.y;
    }

    // get force torque at right ankle and publish
    if (this->rAnkleJoint)
    {
      physics::JointWrench wrench = this->rAnkleJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.r_foot.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.r_foot.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.r_foot.torque.y = wrench.body1Torque.y;
    }

    // get force torque at left wrist and publish
    if (this->lWristJoint)
    {
      physics::JointWrench wrench = this->lWristJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.l_hand.force.x = wrench.body1Force.x;
      this->forceTorqueSensorsMsg.l_hand.force.y = wrench.body1Force.y;
      this->forceTorqueSensorsMsg.l_hand.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.l_hand.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.l_hand.torque.y = wrench.body1Torque.y;
      this->forceTorqueSensorsMsg.l_hand.torque.z = wrench.body1Torque.z;
    }

    // get force torque at right wrist and publish
    if (this->rWristJoint)
    {
      physics::JointWrench wrench = this->rWristJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.r_hand.force.x = wrench.body1Force.x;
      this->forceTorqueSensorsMsg.r_hand.force.y = wrench.body1Force.y;
      this->forceTorqueSensorsMsg.r_hand.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.r_hand.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.r_hand.torque.y = wrench.body1Torque.y;
      this->forceTorqueSensorsMsg.r_hand.torque.z = wrench.body1Torque.z;
    }
    this->pubForceTorqueSensors.publish(this->forceTorqueSensorsMsg);

    gzerr << "  AP FT [" << timer.GetElapsed().Double()*1000.0 << "]\n";
    timer.Start();
#endif

    // populate FromRobot from robot
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->jointStates.position[i] = this->joints[i]->GetAngle(0).Radian();
      this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
      // better to us e GetForceTorque dot joint axis ??
      this->jointStates.effort[i] = this->joints[i]->GetForce(0);
    }
    this->pubJointStates.publish(this->jointStates);

    gzerr << "  AP pub JS [" << timer.GetElapsed().Double()*1000.0 << "]\n";
    timer.Start();

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    /// update pid with feedforward force
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      double q_p =
         this->jointCommands.position[i] - this->jointStates.position[i];

      if (!math::equal(dt, 0.0))
        this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

      this->errorTerms[i].q_p = q_p;

      this->errorTerms[i].qd_p =
         this->jointCommands.velocity[i] - this->jointStates.velocity[i];

      this->errorTerms[i].q_i = math::clamp(
        this->errorTerms[i].q_i + dt * this->errorTerms[i].q_p,
        static_cast<double>(this->jointCommands.i_effort_min[i]),
        static_cast<double>(this->jointCommands.i_effort_max[i]));

      // use gain params to compute force cmd
      double force = this->jointCommands.kp_position[i] *
                     this->errorTerms[i].q_p +
                     this->jointCommands.kp_velocity[i] *
                     this->errorTerms[i].qd_p +
                     this->jointCommands.ki_position[i] *
                     this->errorTerms[i].q_i +
                     this->jointCommands.kd_position[i] *
                     this->errorTerms[i].d_q_p_dt +
                     this->jointCommands.effort[i];

      this->joints[i]->SetForce(0, force);
    }
    this->lastControllerUpdateTime = curTime;

    gzerr << "  AP Control [" << timer.GetElapsed().Double()*1000.0 << "]\n";
    timer.Start();
  }
  gzerr << "AtlasPlugin::UpdateStates [" << timer1.GetElapsed().Double()*1000.0 << "]\n";
}

void AtlasPlugin::OnLContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->lFootContactSensor->GetContacts();


  for (int i = 0; i < contacts.contact_size(); ++i)
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
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
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


  for (int i = 0; i < contacts.contact_size(); ++i)
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
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
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

void AtlasPlugin::foo()
{
}

void AtlasPlugin::RosPubQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosPubQueue.callAvailable(ros::WallDuration(timeout));
    usleep(100);
  }
}
}
