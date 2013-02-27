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

#include <gazebo/transport/Node.hh>

#include "AtlasPlugin.h"

#include "sensor_msgs/Imu.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

////////////////////////////////////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
  this->lFootForce = 0;
  this->lFootTorque = 0;
  this->rFootForce = 0;
  this->rFootTorque = 0;
  // the parent link of the imu_sensor ends up being pelvis after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_senosr block.
  this->imuLinkName = "pelvis";

  this->pelvisLinkName = "pelvis";

  // initialize behavior library
  this->atlasSimInterface = create_atlas_sim_interface();
  this->usingWalkingController = false;
}

////////////////////////////////////////////////////////////////////////////////
AtlasPlugin::~AtlasPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
  // shutdown behavior library
  destroy_atlas_sim_interface();
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();

  // JoitnController: built-in gazebo to control joints
  this->jointController = this->model->GetJointController();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->jointCmdPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + this->model->GetName() + "/joint_cmd");

  // save sdf
  this->sdf = _sdf;

  // initialize update time
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

  // JointController: Publish messages to reset joint controller gains
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    msgs::JointCmd msg;
    msg.set_name(this->joints[i]->GetScopedName()); 
    msg.mutable_position()->set_target(0.0);
    msg.mutable_position()->set_p_gain(0.0);
    msg.mutable_position()->set_i_gain(0.0);
    msg.mutable_position()->set_d_gain(0.0);
    msg.mutable_position()->set_i_max(0.0);
    msg.mutable_position()->set_i_min(0.0);
    msg.mutable_position()->set_limit(0.0);
  }

  this->errorTerms.resize(this->joints.size());
  for (unsigned i = 0; i < this->joints.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i = 0;
    this->errorTerms[i].qd_p = 0;
  }

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

  this->ZeroJointCommands();

  // AtlasSimInterface:  initialize toRobot
  this->toRobot.timestamp = 1.0e9 * this->world->GetSimTime().nsec
    + this->world->GetSimTime().nsec;
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->toRobot.j[i].q_d = 0;
    this->toRobot.j[i].qd_d = 0;
    this->toRobot.j[i].f_d = 0;
    this->toRobot.jparams[i].k_q_p = 0;
    this->toRobot.jparams[i].k_q_i = 0;
    this->toRobot.jparams[i].k_qd_p = 0;
  }

  // AtlasSimInterface:  initialize fromRobot joints data
  this->fromRobot.t = this->world->GetSimTime().Double();
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->fromRobot.j[i].q = 0;
    this->fromRobot.j[i].qd = 0;
    this->fromRobot.j[i].f = 0;
    /*
    this->fromRobot.error_terms[i].q_p = 0;
    this->fromRobot.error_terms[i].qd_p = 0;
    this->fromRobot.error_terms[i].q_i = 0;
    */
  }

  // AtlasSimInterface:  initialize fromRobot sensor data
  this->fromRobot.imu.imu_timestamp = this->toRobot.timestamp;
  this->fromRobot.imu.angular_velocity.n[0] = 0;
  this->fromRobot.imu.angular_velocity.n[1] = 0;
  this->fromRobot.imu.angular_velocity.n[2] = 0;
  this->fromRobot.imu.linear_acceleration.n[0] = 0;
  this->fromRobot.imu.linear_acceleration.n[1] = 0;
  this->fromRobot.imu.linear_acceleration.n[2] = 0;
  this->fromRobot.imu.orientation_estimate.m_qw = 0;
  this->fromRobot.imu.orientation_estimate.m_qx = 0;
  this->fromRobot.imu.orientation_estimate.m_qy = 0;
  this->fromRobot.imu.orientation_estimate.m_qz = 0;
  this->fromRobot.foot_sensors[0].fz = 0;
  this->fromRobot.foot_sensors[0].mx = 0;
  this->fromRobot.foot_sensors[0].my = 0;
  this->fromRobot.foot_sensors[1].fz = 0;
  this->fromRobot.foot_sensors[1].mx = 0;
  this->fromRobot.foot_sensors[1].my = 0;
  this->fromRobot.wrist_sensors[0].f.n[0] = 0;
  this->fromRobot.wrist_sensors[0].f.n[1] = 0;
  this->fromRobot.wrist_sensors[0].f.n[2] = 0;
  this->fromRobot.wrist_sensors[0].m.n[0] = 0;
  this->fromRobot.wrist_sensors[0].m.n[1] = 0;
  this->fromRobot.wrist_sensors[0].m.n[2] = 0;
  this->fromRobot.wrist_sensors[1].f.n[0] = 0;
  this->fromRobot.wrist_sensors[1].f.n[1] = 0;
  this->fromRobot.wrist_sensors[1].f.n[2] = 0;
  this->fromRobot.wrist_sensors[1].m.n[0] = 0;
  this->fromRobot.wrist_sensors[1].m.n[1] = 0;
  this->fromRobot.wrist_sensors[1].m.n[2] = 0;
  // internal debugging use only
  this->fromRobot.pelvis_position.n[0] = 0;
  this->fromRobot.pelvis_position.n[1] = 0;
  this->fromRobot.pelvis_position.n[2] = 0;
  this->fromRobot.pelvis_velocity.n[0] = 0;
  this->fromRobot.pelvis_velocity.n[1] = 0;
  this->fromRobot.pelvis_velocity.n[2] = 0;

  // AtlasSimInterface:
  // Calling into the behavior library to reset controls and set startup
  // behavior.
  this->errorCode = this->atlasSimInterface->reset_control();
  this->errorCode = this->atlasSimInterface->set_desired_behavior("safety");

  // Get imu link
  this->imuLink = this->model->GetLink(this->imuLinkName);
  if (!this->imuLink)
    gzerr << this->imuLinkName << " not found\n";

  // AtlasSimInterface: Get pelvis link for internal debugging only
  this->pelvisLink = this->model->GetLink(this->pelvisLinkName);
  if (!this->pelvisLink)
    gzerr << this->pelvisLinkName << " not found\n";

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
  this->imuSensor =
    boost::shared_dynamic_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::pelvis::"
        "imu_sensor"));
  if (!this->imuSensor)
    gzerr << "imu_sensor not found\n" << "\n";

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

  // initialize imu reference pose
  this->imuOffsetPose = this->imuSensor->GetPose();
  this->imuReferencePose = this->imuOffsetPose + this->imuLink->GetWorldPose();
  this->imuLastLinearVel = imuReferencePose.rot.RotateVector(
    this->imuLink->GetWorldLinearVel());
  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&AtlasPlugin::DeferredLoad, this));
}


////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->jointCommands.header.stamp = _msg->header.stamp;

  if (_msg->position.size() == this->jointCommands.position.size())
    std::copy(_msg->position.begin(), _msg->position.end(),
      this->jointCommands.position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg->position.size(), this->jointCommands.position.size());

  if (_msg->velocity.size() == this->jointCommands.velocity.size())
    std::copy(_msg->velocity.begin(), _msg->velocity.end(),
      this->jointCommands.velocity.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg->velocity.size(), this->jointCommands.velocity.size());

  if (_msg->effort.size() == this->jointCommands.effort.size())
    std::copy(_msg->effort.begin(), _msg->effort.end(),
      this->jointCommands.effort.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->jointCommands.effort.size());

  if (_msg->kp_position.size() == this->jointCommands.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->jointCommands.kp_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->jointCommands.kp_position.size());

  if (_msg->ki_position.size() == this->jointCommands.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->jointCommands.ki_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->jointCommands.ki_position.size());

  if (_msg->kd_position.size() == this->jointCommands.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->jointCommands.kd_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->jointCommands.kd_position.size());

  if (_msg->kp_velocity.size() == this->jointCommands.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->jointCommands.kp_velocity.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->jointCommands.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->jointCommands.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->jointCommands.i_effort_min.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->jointCommands.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->jointCommands.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->jointCommands.i_effort_max.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->jointCommands.i_effort_max.size());
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // Get window size from ros parameter server (seconds)
  if (!this->rosNode->getParam(
    "atlas_controller/statistics_time_window_size",
    this->jointCommandsAgeBufferDuration))
  {
    this->jointCommandsAgeBufferDuration = 1.0;
    ROS_INFO("controller statistics window size not specified in"
             " ros parameter server, defaulting to %f sec.",
             this->jointCommandsAgeBufferDuration);
  }
  double stepSize = this->world->GetPhysicsEngine()->GetStepTime();
  if (math::equal(stepSize, 0.0))
  {
    stepSize = 0.001;
    ROS_WARN("simulation step size is zero, something is wrong,"
              "  Defaulting to step size of %f sec.", stepSize);
  }
  // document this from
  // http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  // Online algorithm
  // where Delta2 buffer contains delta*(x - mean) line from code block
  unsigned int bufferSize = this->jointCommandsAgeBufferDuration / stepSize;
  this->jointCommandsAgeBuffer.resize(bufferSize);
  this->jointCommandsAgeDelta2Buffer.resize(bufferSize);
  this->jointCommandsAgeBufferIndex = 0;
  this->jointCommandsAgeMean = 0.0;
  this->jointCommandsAgeVariance = 0.0;

  // ROS Controller API
  /// brief broadcasts the robot states
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "atlas/joint_states", 1);

  this->pubForceTorqueSensors =
    this->rosNode->advertise<atlas_msgs::ForceTorqueSensors>(
    "atlas/force_torque_sensors", 10);

  // ros publication / subscription
  this->pubControllerStatistics =
    this->rosNode->advertise<atlas_msgs::ControllerStatistics>(
    "atlas/controller_statistics", 10);

  this->pubLFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/l_foot_contact", 10);

  this->pubRFootContact =
    this->rosNode->advertise<geometry_msgs::Wrench>(
      "atlas/r_foot_contact", 10);

  // ros topic subscribtions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "atlas/joint_commands", 1,
    boost::bind(&AtlasPlugin::SetJointCommands, this, _1),
    ros::VoidPtr(), &this->rosQueue);

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint commands, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointCommandsSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  this->subJointCommands=
    this->rosNode->subscribe(jointCommandsSo);

  // publish imu data
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>("atlas/imu", 10);

  // initialize status pub time
  this->lastControllerStatisticsTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // AtlasSimInterface:
  // subscribe to a control_mode string message, current valid commands are:
  //   walk, stand, safety, stand-prep, none
  // the command is passed to the AtlasSimInterface library.
  ros::SubscribeOptions atlasControlModeSo =
    ros::SubscribeOptions::create<std_msgs::String>(
    "atlas/control_mode", 100,
    boost::bind(&AtlasPlugin::OnRobotMode, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subAtlasControlMode = this->rosNode->subscribe(atlasControlModeSo);

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&AtlasPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&AtlasPlugin::UpdateStates, this));

  // on contact
  this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnLContactUpdate, this));

  this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnRContactUpdate, this));
}

// AtlasSimInterface:
// subscribe to a control_mode string message, current valid commands are:
//   walk, stand, safety, stand-prep, none
// the command is passed to the AtlasSimInterface library.
void AtlasPlugin::OnRobotMode(const std_msgs::String::ConstPtr &_mode)
{
  // to make it stand
  //  * stand-prep:  puts robot in standing pose while harnessed
  //  * remove the harness
  //  * after robot hits ground, switch over to stand mode
  //  * robot should dynamically balance itself

  // simple state machine here to do something
  if (_mode->data == "safety" || _mode->data == "stand-prep" ||
      _mode->data == "stand" || _mode->data == "walk")
  {
    // start AtlasSimLibrary controller
    // this mode resets the timer, and automatically goes into stand mode
    // after 
    this->usingWalkingController = true;
    this->atlasSimInterface->set_desired_behavior(_mode->data);
    this->ZeroJointCommands();
  }
  else if (_mode->data == "none")
  {
    // revert to PID control
    this->LoadPIDGainsFromParameter();
    this->usingWalkingController = false;
    this->atlasSimInterface->set_desired_behavior(_mode->data);
  }
  else
  {
    ROS_WARN("Unknown robot mode [%s]", _mode->data.c_str());
  }
}

void AtlasPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // AtlasSimInterface:
    // populate fromRobot from robot
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->fromRobot.t = curTime.Double();
      this->fromRobot.j[i].q = this->joints[i]->GetAngle(0).Radian();
      this->fromRobot.j[i].qd = this->joints[i]->GetVelocity(0);
      // wait to fill in this->fromRobot.j[i].f later
    }

    // get imu data from imu link
    if (this->imuLink && curTime > this->lastImuTime)
    {
      double dt = (curTime - this->lastImuTime).Double();
      // Get imuLnk Pose/Orientation
      math::Pose parentEntityPose = this->imuLink->GetWorldPose();
      math::Pose imuLinkPose = this->imuOffsetPose + parentEntityPose;

      // calculate imu's linear velocity in imu frame
      math::Vector3 imuAngularVelParentFrame =
        parentEntityPose.rot.GetInverse().RotateVector(
        this->imuLink->GetWorldAngularVel());
      math::Vector3 imuLinearVelParentFrame =
        parentEntityPose.rot.GetInverse().RotateVector(
        this->imuLink->GetWorldLinearVel()) +
        this->imuOffsetPose.pos.Cross(imuAngularVelParentFrame);
      math::Vector3 imuLinearVel =
        this->imuOffsetPose.rot.GetInverse().RotateVector(
          imuLinearVelParentFrame);

      sensor_msgs::Imu imuMsg;
      imuMsg.header.frame_id = this->imuLinkName;
      imuMsg.header.stamp = ros::Time(curTime.Double());

      // compute angular rates
      {
        // get world twist and convert to local frame
        math::Vector3 wLocal = imuLinkPose.rot.GetInverse().RotateVector(
          this->imuLink->GetWorldAngularVel());
        imuMsg.angular_velocity.x = wLocal.x;
        imuMsg.angular_velocity.y = wLocal.y;
        imuMsg.angular_velocity.z = wLocal.z;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.angular_velocity.n[0] = wLocal.x;
        this->fromRobot.imu.angular_velocity.n[1] = wLocal.y;
        this->fromRobot.imu.angular_velocity.n[2] = wLocal.z;
      }

      // compute acceleration
      {
        math::Vector3 accel = (imuLinearVel - this->imuLastLinearVel)/dt;

        imuMsg.linear_acceleration.x = accel.x;
        imuMsg.linear_acceleration.y = accel.y;
        imuMsg.linear_acceleration.z = accel.z;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.linear_acceleration.n[0] = accel.x;
        this->fromRobot.imu.linear_acceleration.n[1] = accel.y;
        this->fromRobot.imu.linear_acceleration.n[2] = accel.z;

        this->imuLastLinearVel = imuLinearVel;
      }

      // compute orientation
      {
        // Get IMU rotation relative to Initial IMU Reference Pose
        math::Quaternion imuRot =
          imuLinkPose.rot * this->imuReferencePose.rot.GetInverse();

        imuMsg.orientation.x = imuRot.x;
        imuMsg.orientation.y = imuRot.y;
        imuMsg.orientation.z = imuRot.z;
        imuMsg.orientation.w = imuRot.w;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.orientation_estimate.m_qw = imuRot.w;
        this->fromRobot.imu.orientation_estimate.m_qx = imuRot.x;
        this->fromRobot.imu.orientation_estimate.m_qy = imuRot.y;
        this->fromRobot.imu.orientation_estimate.m_qz = imuRot.z;
      }

      this->pubImu.publish(imuMsg);

      // update time
      this->lastImuTime = curTime.Double();
    }

    // AtlasSimInterface: pelvis pose/twist for internal debugging only.
    if (this->pelvisLink && curTime > this->lastImuTime)
    {
      math::Pose pose = this->pelvisLink->GetWorldPose();
      math::Vector3 vel = this->pelvisLink->GetWorldLinearVel();

      /// WARNING: these are inertial?
      this->fromRobot.pelvis_position.n[0] = pose.pos.x;
      this->fromRobot.pelvis_position.n[1] = pose.pos.y;
      this->fromRobot.pelvis_position.n[2] = pose.pos.z;
      this->fromRobot.pelvis_velocity.n[0] = vel.x;
      this->fromRobot.pelvis_velocity.n[1] = vel.y;
      this->fromRobot.pelvis_velocity.n[2] = vel.z;
    }

    this->forceTorqueSensorsMsg.header.stamp =
      ros::Time(curTime.sec, curTime.nsec);

    // get force torque at left ankle and publish
    if (this->lAnkleJoint)
    {
      physics::JointWrench wrench = this->lAnkleJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.l_foot.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.l_foot.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.l_foot.torque.y = wrench.body1Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in fromRobot
      this->fromRobot.foot_sensors[0].fz = wrench.body1Force.z;
      this->fromRobot.foot_sensors[0].mx = wrench.body1Torque.x;
      this->fromRobot.foot_sensors[0].my = wrench.body1Torque.y;
    }

    // get force torque at right ankle and publish
    if (this->rAnkleJoint)
    {
      physics::JointWrench wrench = this->rAnkleJoint->GetForceTorque(0);
      this->forceTorqueSensorsMsg.r_foot.force.z = wrench.body1Force.z;
      this->forceTorqueSensorsMsg.r_foot.torque.x = wrench.body1Torque.x;
      this->forceTorqueSensorsMsg.r_foot.torque.y = wrench.body1Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in fromRobot
      this->fromRobot.foot_sensors[1].fz = wrench.body1Force.z;
      this->fromRobot.foot_sensors[1].mx = wrench.body1Torque.x;
      this->fromRobot.foot_sensors[1].my = wrench.body1Torque.y;
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

      // AtlasSimInterface: populate wrist force torque sensor in fromRobot
      this->fromRobot.wrist_sensors[0].f.n[0] = wrench.body1Force.x;
      this->fromRobot.wrist_sensors[0].f.n[1] = wrench.body1Force.y;
      this->fromRobot.wrist_sensors[0].f.n[2] = wrench.body1Force.z;
      this->fromRobot.wrist_sensors[0].m.n[0] = wrench.body1Torque.x;
      this->fromRobot.wrist_sensors[0].m.n[1] = wrench.body1Torque.y;
      this->fromRobot.wrist_sensors[0].m.n[2] = wrench.body1Torque.z;
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

      // AtlasSimInterface: populate wrist force torque sensor in fromRobot
      this->fromRobot.wrist_sensors[1].f.n[0] = wrench.body1Force.x;
      this->fromRobot.wrist_sensors[1].f.n[1] = wrench.body1Force.y;
      this->fromRobot.wrist_sensors[1].f.n[2] = wrench.body1Force.z;
      this->fromRobot.wrist_sensors[1].m.n[0] = wrench.body1Torque.x;
      this->fromRobot.wrist_sensors[1].m.n[1] = wrench.body1Torque.y;
      this->fromRobot.wrist_sensors[1].m.n[2] = wrench.body1Torque.z;
    }
    this->pubForceTorqueSensors.publish(this->forceTorqueSensorsMsg);

    // populate jointStates from robot
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->jointStates.position[i] = this->joints[i]->GetAngle(0).Radian();
      this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
      // but wait on publish until we've determined the force to apply
    }

    // AtlasSimInterface:
    if (this->usingWalkingController)
      // process data fromRobot to create output data toRobot
      this->errorCode = this->atlasSimInterface->process_control_input(
        this->fromRobot, this->toRobot);

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    {
      boost::mutex::scoped_lock lock(this->mutex);
      {
        // Keep track of age of jointCommands age in seconds.
        // Note the value is invalid as a moving window average age
        // until the buffer is full.
        this->jointCommandsAge = curTime.Double() -
          this->jointCommands.header.stamp.toSec();

        double weightedJointCommandsAge = this->jointCommandsAge
          / this->jointCommandsAgeBuffer.size();

        // for variance calculation, save delta before average is updated.
        double delta = this->jointCommandsAge - this->jointCommandsAgeMean;

        // update average
        this->jointCommandsAgeMean += weightedJointCommandsAge;
        this->jointCommandsAgeMean -=
          this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex];

        // update variance with new average
        double delta2 = delta *
          (this->jointCommandsAge - this->jointCommandsAgeMean);
        this->jointCommandsAgeVariance += delta2;
        this->jointCommandsAgeVariance -= 
          this->jointCommandsAgeDelta2Buffer[
          this->jointCommandsAgeBufferIndex];

        // save weighted average in window
        this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex] =
          weightedJointCommandsAge;

        // save delta buffer for incremental variance calculation
        this->jointCommandsAgeDelta2Buffer[
          this->jointCommandsAgeBufferIndex] = delta2;

        this->jointCommandsAgeBufferIndex =
         (this->jointCommandsAgeBufferIndex + 1) %
         this->jointCommandsAgeBuffer.size();
      }

      /// update pid with feedforward force
      for (unsigned int i = 0; i < this->joints.size(); ++i)
      {
        // truncate joint position within range of motion
        double positionTarget = math::clamp(
          this->jointCommands.position[i],
          this->joints[i]->GetLowStop(0).Radian(),
          this->joints[i]->GetHighStop(0).Radian());

        double q_p = positionTarget - this->jointStates.position[i];

        if (!math::equal(dt, 0.0))
          this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

        this->errorTerms[i].q_p = q_p;

        this->errorTerms[i].qd_p =
           this->jointCommands.velocity[i] - this->jointStates.velocity[i];

        if (!math::equal(this->jointCommands.ki_position[i], 0.0))
          this->errorTerms[i].q_i = math::clamp(
            this->errorTerms[i].q_i + dt * this->errorTerms[i].q_p,
            static_cast<double>(this->jointCommands.i_effort_min[i]) /
            this->jointCommands.ki_position[i],
            static_cast<double>(this->jointCommands.i_effort_max[i]) /
            this->jointCommands.ki_position[i]);

        // use gain params to compute force cmd
        double force =
          this->jointCommands.kp_position[i] * this->errorTerms[i].q_p +
          this->jointCommands.ki_position[i] * this->errorTerms[i].q_i +
          this->jointCommands.kd_position[i] * this->errorTerms[i].d_q_p_dt +
          this->jointCommands.kp_velocity[i] * this->errorTerms[i].qd_p +
          this->jointCommands.effort[i];


        // AtlasSimInterface:  add controller force to overall control torque.
        force += this->toRobot.j[i].f_d;

        this->joints[i]->SetForce(0, force);

        // fill in jointState efforts
        this->jointStates.effort[i] = force;

        // AtlasSimInterface: fill in fromRobot efforts.
        // FIXME: Is this used by the controller?  i.e. should this happen
        // before process_control_input?
        this->fromRobot.j[i].f = force;
      }
    }

    this->lastControllerUpdateTime = curTime;

    this->pubJointStates.publish(this->jointStates);

    /// controller statistics diagnostics, damages, etc.
    if (this->pubControllerStatistics.getNumSubscribers() > 0)
    {
      if ((curTime - this->lastControllerStatisticsTime).Double() >=
        1.0/this->updateRate)
      {
        atlas_msgs::ControllerStatistics msg;
        msg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
        msg.command_age = this->jointCommandsAge;
        msg.command_age_mean = this->jointCommandsAgeMean;
        msg.command_age_variance = this->jointCommandsAgeVariance /
          (this->jointCommandsAgeBuffer.size() - 1);
        msg.command_age_window_size = this->jointCommandsAgeBufferDuration;

        this->pubControllerStatistics.publish(msg);
        this->lastControllerStatisticsTime = curTime;
      }
    }
  }
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

void AtlasPlugin::ZeroJointCommands()
{
  for (unsigned i = 0; i < this->jointCommands.name.size(); ++i)
  {
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
}

void AtlasPlugin::LoadPIDGainsFromParameter()
{
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
}

void AtlasPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}
