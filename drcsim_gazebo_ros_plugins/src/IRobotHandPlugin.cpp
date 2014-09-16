/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "drcsim_gazebo_ros_plugins/IRobotHandPlugin.h"

////////////////////////////////////////////////////////////////////////////////
IRobotHandPlugin::IRobotHandPlugin()
{
  for (int i = 0; i < 5; ++i)
  {
    this->kp_position[i] = 1.0;
    this->ki_position[i]  = 0.0;
    this->kd_position[i]  = 0.0;
    this->i_position_effort_min[i] = 0.0;
    this->i_position_effort_max[i] = 0.0;
    this->kp_velocity[i]  = 0.1;
    this->ki_velocity[i]  = 0.0;
    this->kd_velocity[i]  = 0.0;
    this->i_velocity_effort_min[i] = 0.0;
    this->i_velocity_effort_max[i] = 0.0;
  }

  this->errorTerms.resize(5);  // hand has 5 DOF
  for (unsigned i = 0; i < 5; ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
IRobotHandPlugin::~IRobotHandPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->sdf = _sdf;

  if(!this->sdf->HasElement("side") || 
     !this->sdf->GetElement("side")->GetValue()->Get(this->side) ||
     ((this->side != "left") && (this->side != "right")))
  {
    gzerr << "Failed to determine which hand we're controlling; "
             "aborting plugin load." << std::endl;
    return;
  }

  gzlog << "IRobotHandPlugin loading for " << this->side << " hand." << 
    std::endl;

  if(!this->FindJoints())
    return;

  this->SetJointSpringDamper();

  // save thumb upper limit
  this->thumbUpperLimit = this->fingerBaseJoints[2]->GetUpperLimit(0).Radian();
  this->thumbAntagonistAngle = 0.0;

  // Load ROS
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

  // publish multi queue
  this->pmq.startServiceThread();

  // ros publication / subscription
  /// brief broadcasts the robot states
  this->pubJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
  if (this->side == "left")
    this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
      "irobot_hands/l_hand/joint_states", 10);
  else if (this->side == "right")
    this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
      "irobot_hands/r_hand/joint_states", 10);

  // broadcasts handle state
  std::string sensorStr = this->side + "_hand/sensors/raw";
  this->pubHandleStateQueue = this->pmq.addPub<handle_msgs::HandleSensors>();
  this->pubHandleState = this->rosNode->advertise<handle_msgs::HandleSensors>(
    sensorStr, 100, true);

  // subscribe to user published handle control commands
  std::string controlStr = this->side + "_hand/control";
  ros::SubscribeOptions handleCommandSo =
    ros::SubscribeOptions::create<handle_msgs::HandleControl>(
    controlStr, 100,
    boost::bind(&IRobotHandPlugin::SetHandleCommand, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  // Enable TCP_NODELAY since TCP causes bursty communication with high jitter,
  handleCommandSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  this->subHandleCommand =
    this->rosNode->subscribe(handleCommandSo);

  // controller time control
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // start callback queue
  this->callbackQueeuThread = boost::thread(
    boost::bind(&IRobotHandPlugin::RosQueueThread, this));

  // connect to gazebo world update
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
     boost::bind(&IRobotHandPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::SetHandleCommand(
  const handle_msgs::HandleControl::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->controlMutex);
  // Notee:
  // this->handleCommand.value[i]
  // where indices are:
  // 0: index finger flex
  // 1: middle finger flex
  // 2: thumb finger flex
  // 3: thumb finger antagonist
  // 4: index / middle finger spread

  // Update handleCommand
  for (int i = 0; i < 5; ++i)
  {
    // this->handleCommand.type[i] = handle_msgs::HandleControl::VELOCITY;
    // this->handleCommand.type[i] = handle_msgs::HandleControl::POSITION;
    // this->handleCommand.type[i] = handle_msgs::HandleControl::CURRENT;
    // this->handleCommand.type[i] = handle_msgs::HandleControl::VOLTAGE;
    this->handleCommand.type[i] = _msg->type[i];
    this->handleCommand.value[i] = _msg->value[i];
    this->handleCommand.valid[i] = _msg->valid[i];

    /// \TODO: HandleControlValueToJointAngle
    /// HandleControl::value is the motor angle
    /// motor angle --> spool length
    /// spool length --> overall joint angle
  }
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::UpdateStates()
{
  gazebo::common::Time curTime = this->world->GetSimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // gather robot state data and publish them
    this->GetAndPublishHandleState(curTime);

    this->UpdatePIDControl(
      (curTime - this->lastControllerUpdateTime).Double());

    this->lastControllerUpdateTime = curTime;
  }
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::GetAndPublishHandleState(
  const gazebo::common::Time &_curTime)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  this->handleState.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

  this->handleState.motorHallEncoder[0] = 0;  // int32
  this->handleState.motorHallEncoder[1] = 0;  // int32
  this->handleState.motorHallEncoder[2] = 0;  // int32
  this->handleState.motorHallEncoder[3] = 0;  // int32

  this->handleState.motorWindingTemp[0] = 0.0;
  this->handleState.motorWindingTemp[1] = 0.0;
  this->handleState.motorWindingTemp[2] = 0.0;
  this->handleState.motorWindingTemp[3] = 0.0;

  this->handleState.airTemp = 0.0;

  this->handleState.motorVelocity[0] = 0;  // int32
  this->handleState.motorVelocity[1] = 0;  // int32
  this->handleState.motorVelocity[2] = 0;  // int32
  this->handleState.motorVelocity[3] = 0;  // int32

  this->handleState.motorHousingTemp[0] = 0.0;
  this->handleState.motorHousingTemp[1] = 0.0;
  this->handleState.motorHousingTemp[2] = 0.0;
  this->handleState.motorHousingTemp[3] = 0.0;
  this->handleState.motorHousingTemp[4] = 0.0;

  this->handleState.motorCurrent[0] = 0.0;
  this->handleState.motorCurrent[1] = 0.0;
  this->handleState.motorCurrent[2] = 0.0;
  this->handleState.motorCurrent[3] = 0.0;
  this->handleState.motorCurrent[4] = 0.0;

  for(int i = 0; i < 3; ++i)
  {
    this->handleState.fingerTactile[i].proximal.resize(0);  // float32[]
    this->handleState.fingerTactile[i].distal.resize(0);
    this->handleState.fingerTactileTemp[i].proximal.resize(0);
    this->handleState.fingerTactileTemp[i].distal.resize(0);
  }

  this->handleState.fingerSpread = 0;  // int32

  this->handleState.proximalJointAngle[0] = 0;  // int32
  this->handleState.proximalJointAngle[1] = 0;  // int32
  this->handleState.proximalJointAngle[2] = 0;  // int32

  for(int i = 0; i < 3; ++i)
  {
    this->handleState.distalJointAngle[i].proximal.resize(0);  // float32[]
    this->handleState.distalJointAngle[i].distal.resize(0);

    this->handleState.proximalAcceleration[i].x = 0.0;
    this->handleState.proximalAcceleration[i].y = 0.0;
    this->handleState.proximalAcceleration[i].z = 0.0;

    this->handleState.distalAcceleration[i].x = 0.0;
    this->handleState.distalAcceleration[i].y = 0.0;
    this->handleState.distalAcceleration[i].z = 0.0;
  }

  for(int i = 0; i < 12; ++i)
  {
    this->handleState.responses[i] = false;  // bool
    this->handleState.responseHistory[i] = 0;  // int8
  }

  for(int i = 0; i < 5; ++i)
  {
    this->handleState.motorError[i] = 0;  // int16
  }

  // publish robot states
  this->pubHandleStateQueue->push(this->handleState, this->pubHandleState);


  // setup and publish hands joint states
  int njs = 0;

  // setup hands joint states
  this->jointStates.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

  // base rotation joint of finger
  this->jointStates.name[njs] =
    this->fingerBaseRotationJoints[0]->GetName();
  this->jointStates.position[njs] =
    this->fingerBaseRotationJoints[0]->GetAngle(0).Radian();
  this->jointStates.velocity[njs] =
    this->fingerBaseRotationJoints[0]->GetVelocity(0);
  this->jointStates.effort[njs] =
    this->fingerBaseRotationJoints[0]->GetForce(0);
  ++njs;

  // base rotation joint of right finger
  this->jointStates.name[njs] =
    this->fingerBaseRotationJoints[1]->GetName();
  this->jointStates.position[njs] =
    this->fingerBaseRotationJoints[1]->GetAngle(0).Radian();
  this->jointStates.velocity[njs] =
    this->fingerBaseRotationJoints[1]->GetVelocity(0);
  this->jointStates.effort[njs] =
    this->fingerBaseRotationJoints[1]->GetForce(0);
  ++njs;

  // base joint flex
  for (unsigned int i = 0; i < 3; ++i)
  {
    this->jointStates.name[njs] =
      this->fingerBaseJoints[i]->GetName();
    this->jointStates.position[njs] =
      this->fingerBaseJoints[i]->GetAngle(0).Radian();
    this->jointStates.velocity[njs] =
      this->fingerBaseJoints[i]->GetVelocity(0);
    this->jointStates.effort[njs] =
      this->fingerBaseJoints[i]->GetForce(0u);
    ++njs;
  }

  // flexure joints
  for (int nFinger = 0; nFinger < 3; ++nFinger)
  {
    int numFlex = this->flexureFlexJoints[nFinger].size();
    for (int nFlexure = 0; nFlexure < numFlex; ++nFlexure)
    {
      // flexure flex joints
      this->jointStates.name[njs] =
        this->flexureFlexJoints[nFinger][nFlexure]->GetName();
      this->jointStates.position[njs] =
        this->flexureFlexJoints[nFinger][nFlexure]->GetAngle(0).Radian();
      this->jointStates.velocity[njs] =
        this->flexureFlexJoints[nFinger][nFlexure]->GetVelocity(0);
      this->jointStates.effort[njs] =
        this->flexureFlexJoints[nFinger][nFlexure]->GetForce(0u);
      ++njs;

      // flexure twist joints
      this->jointStates.name[njs] =
        this->flexureTwistJoints[nFinger][nFlexure]->GetName();
      this->jointStates.position[njs] =
        this->flexureTwistJoints[nFinger][nFlexure]->GetAngle(0).Radian();
      this->jointStates.velocity[njs] =
        this->flexureTwistJoints[nFinger][nFlexure]->GetVelocity(0);
      this->jointStates.effort[njs] =
        this->flexureTwistJoints[nFinger][nFlexure]->GetForce(0u);
      ++njs;
    }
  }

  // publish joint states
  this->pubJointStatesQueue->push(this->jointStates,
    this->pubJointStates);
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::UpdatePIDControl(double _dt)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  /// update thumb antagonist angle
  {
    // antagonist angle is between 0 (no antagonist) and
    // upper - lower (pinned to lower position).
    this->thumbAntagonistAngle =
      std::max(0.0,
      std::min(this->thumbUpperLimit -
               this->fingerBaseJoints[2]->GetLowerLimit(0).Radian(),
               this->HandleControlFlexValueToFlexJointAngle(
               this->handleCommand.value[3])));

    // set thum uppper limit according to antagonist angle
    this->fingerBaseJoints[2]->SetUpperLimit(
      0, this->thumbUpperLimit - this->thumbAntagonistAngle);

    // debug
    // ROS_ERROR("%s lower %f upper %f antag %f upper %f", this->side.c_str(),
    //   this->fingerBaseJoints[2]->GetLowerLimit(0).Radian(),
    //   this->thumbUpperLimit, this->thumbAntagonistAngle,
    //   this->fingerBaseJoints[2]->GetUpperLimit(0).Radian());
  }

  for (int j = 0; j < 4; ++j)
  {
    /// control index
    ///   j == 0: index finger flex
    ///   j == 1: middle finger flex
    ///   j == 2: thumb flex
    ///   j == 3: skip: antagonist angle setting
    ///   j == 4: index / middle finger spread
    if (j == 3)
      j = 4;   // skip to spread.  antagonist is taken care of separately.

    double torque;
    double target;
    double current;

    if (j == 4)  // spread target
      target = this->HandleControlSpreadValueToSpreadJointAngle(
        this->handleCommand.value[j]);
    else  // flex target
      target = this->HandleControlFlexValueToFlexJointAngle(
        this->handleCommand.value[j]);

    int numFlex = this->flexureFlexJoints[j].size();

    // Get current finger state
    double baseJointPos = 0;
    double baseJointVel = 0;
    double flexureFlexJointPos = 0;
    double flexureFlexJointVel = 0;
    double currentPos = 0;
    double currentVel = 0;
    if (j < 4)  // sum finger joint angle for baseJoint and flexureFlexJoint
    {
      baseJointPos = this->fingerBaseJoints[j]->GetAngle(0).Radian();
      baseJointVel = this->fingerBaseJoints[j]->GetVelocity(0);
      flexureFlexJointPos = 0;
      flexureFlexJointVel = 0;
      for (int i = 0; i < numFlex; ++i)
      {
        flexureFlexJointPos +=
          this->flexureFlexJoints[j][i]->GetAngle(0).Radian();
        flexureFlexJointVel +=
          this->flexureFlexJoints[j][i]->GetVelocity(0);
      }
      // compute overall flex from baseJoint and flexureFlex joint positions
      currentPos = baseJointPos + flexureFlexJointPos;
      currentVel = baseJointVel + flexureFlexJointVel;
      /// \TODO: should we limit target based on baseJointPos
      /// or flexureFlexJointPos?
    }
    else
    {
      // compute spread position
      currentPos =
        this->fingerBaseRotationJoints[0]->GetAngle(0).Radian() +
        this->fingerBaseRotationJoints[1]->GetAngle(0).Radian();
      currentVel =
        this->fingerBaseRotationJoints[0]->GetVelocity(0) +
        this->fingerBaseRotationJoints[1]->GetVelocity(0);
    }


    double kp, ki, kd, i_effort_max, i_effort_min;

    if (this->handleCommand.type[j] == handle_msgs::HandleControl::POSITION)
    {
      // set state  for position control
      current = currentPos;

      kp = this->kp_position[j];
      ki = this->ki_position[j];
      kd = this->kd_position[j];
      i_effort_min = this->i_position_effort_min[j];
      i_effort_max = this->i_position_effort_max[j];
    }
    else if (this->handleCommand.type[j] ==
             handle_msgs::HandleControl::VELOCITY)
    {
      // set state for velocity control
      /// \TODO: figure out a good limit for the combined finger joint angle.
      current = currentVel;

      kp = this->kp_velocity[j];
      ki = this->ki_velocity[j];
      kd = this->kd_velocity[j];
      i_effort_min = this->i_velocity_effort_min[j];
      i_effort_max = this->i_velocity_effort_max[j];

      // stop driving the finger if we are over the max angle
      // const double maxSimStableFingerPos = M_PI;
      // if (currentPos > maxSimStableFingerPos)
      //   target = 0;
    }
    else if (this->handleCommand.type[j] == handle_msgs::HandleControl::CURRENT)
    {
      ROS_ERROR("Control Type [CURRENT] not available");
      return;
    }
    else if (this->handleCommand.type[j] == handle_msgs::HandleControl::VOLTAGE)
    {
      ROS_ERROR("Control Type [VOLTAGE] not available");
      return;
    }
    else if (this->handleCommand.type[j] == handle_msgs::HandleControl::ANGLE)
    {
      ROS_ERROR("Control Type [ANGLE] not available");
      /// \TODO: how to convert int32 to angle?
      return;
    }
    else if (this->handleCommand.type[j] == 0)
    {
      // uncontrolled
      return;
    }
    else
    {
      ROS_ERROR("Control Type [%d] not available", this->handleCommand.type[j]);
      return;
    }

    // calculate control torque / force
    {
      double q_p = target - current;

      this->errorTerms[j].q_p = q_p;
      if (!gazebo::math::equal(_dt, 0.0))
        this->errorTerms[j].d_q_p_dt = (q_p - this->errorTerms[j].q_p) / _dt;
      this->errorTerms[j].q_i = gazebo::math::clamp(
        this->errorTerms[j].q_i + _dt * this->errorTerms[j].q_p,
        i_effort_min, i_effort_max);

      // use gain params to compute force cmd
      torque = kp * this->errorTerms[j].q_p +
               ki * this->errorTerms[j].q_i +
               kd * this->errorTerms[j].d_q_p_dt;
    }


    if (j < 4)  // if flex control (not spread)
    {
      // tend can only transmit tension, not compression
      double tendonTorque = std::max(0.0, torque);

      // For thumb, apply only if current angle of the base joint
      // is less than its antagonist angle.
      if (j == 2 && this->fingerBaseJoints[j]->GetAngle(0) >
          this->thumbUpperLimit - this->thumbAntagonistAngle)
      {
        this->fingerBaseJoints[j]->SetForce(0, 0);
      }
      else
      {
        // hack: increase damping coefficient to reduce jitter and increase
        // grasp stability
        double damping = 0.5*tendonTorque;
        this->fingerBaseJoints[j]->SetStiffnessDamping(0u,
          this->fingerBaseJoints[j]->GetStiffness(0u),
          damping);

        this->fingerBaseJoints[j]->SetForce(0, std::max(0.0, tendonTorque/2.0));
      }
      for (int i = 0; i < numFlex; ++i)
      {
        // hack: increase damping coefficient to reduce jitter and increase
        // grasp stability
        double damping = 0.5*tendonTorque/numFlex;
        this->flexureFlexJoints[j][i]->SetStiffnessDamping(0u,
          this->flexureFlexJoints[j][i]->GetStiffness(0u),
          damping);

        this->flexureFlexJoints[j][i]->SetForce(0, (tendonTorque/2.0)/numFlex);
      }
    }
    else  // control spread
    {
      // hack: increase damping coefficient to reduce jitter and increase
      // grasp stability
      double damping = torque;
      this->fingerBaseRotationJoints[0]->SetStiffnessDamping(0u,
        this->fingerBaseRotationJoints[0]->GetStiffness(0u),
        damping);

      /// update index/middle finger spread
      this->fingerBaseRotationJoints[0]->SetForce(0, torque);
      // setting one joint is enough due to gearbox
      // this->fingerBaseRotationJoints[1]->SetForce(0, torque);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
bool IRobotHandPlugin::GetAndPushBackJoint(const std::string& _joint_name,
                                           gazebo::physics::Joint_V& _joints)
{
  gazebo::physics::JointPtr joint = this->model->GetJoint(_joint_name);
  if(!joint)
  {
    gzerr << "Failed to find joint: " << _joint_name << 
      "; aborting plugin load." << std::endl;
    return false;
  }
  _joints.push_back(joint);
  gzlog << "IRobotHandPlugin found joint: " << _joint_name << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::KpKdToCFMERP(const double _dt,
                           const double _kp, const double _kd,
                           double &_cfm, double &_erp)
{
  /// \TODO: check for NaN cases
  _erp = _dt * _kp / ( _dt * _kp + _kd );
  _cfm = 1.0 / ( _dt * _kp + _kd );
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::CFMERPToKpKd(const double _dt,
                           const double _cfm, const double _erp,
                           double &_kp, double &_kd)
{
  /// \TODO: check for NaN cases
  _kp = _erp / (_dt * _cfm);
  _kd = (1.0 - _erp) / _cfm;
}

////////////////////////////////////////////////////////////////////////////////
double IRobotHandPlugin::HandleControlFlexValueToFlexJointAngle(int _value)
{
  // convert from int32 to joint angle in radians

  // _value is taken as motor rotation in radians.
  // in practice, spool diameter varies from 10mm to 14mm.
  const double intToMotorAngle = 1.0;
  double motorAngle = intToMotorAngle * static_cast<double>(_value);
  const double spoolDiameter = 0.012;
  double tendonLength = spoolDiameter * motorAngle;
  const double tendonLengthToAngle = 0.01;  // wild guess
  double jointAngle = tendonLengthToAngle * tendonLength;
  return jointAngle;
}

////////////////////////////////////////////////////////////////////////////////
double IRobotHandPlugin::HandleControlSpreadValueToSpreadJointAngle(int _value)
{
  // convert from int32 to joint angle in radians
  /// \TODO: figure out what is the unit of incoming integer.
  const double intToMotorAngle = 1.0;
  double motorAngle = intToMotorAngle * static_cast<double>(_value);
  const double reductionRatio = 1.0/1000.0;
  return reductionRatio * motorAngle;
}

////////////////////////////////////////////////////////////////////////////////
bool IRobotHandPlugin::FindJoints()
{
  this->flexureTwistJoints.resize(this->numFingers);
  this->flexureFlexJoints.resize(this->numFingers);

  // Load up the joints we expect to use, finger by finger.
  gazebo::physics::JointPtr joint;
  char joint_name[256];
  for(int f=0; f<this->numFingers; ++f)
  {
    // Get the base rotation joint (only fingers 0 and 1)
    if((f==0) || (f==1))
    {
      snprintf(joint_name, sizeof(joint_name),
        "%s_finger[%d]/joint_base_rotation",
        this->side.c_str(), f);
      if(!this->GetAndPushBackJoint(joint_name, 
            this->fingerBaseRotationJoints))
        return false;
    }

    // Get the base joint
    snprintf(joint_name, sizeof(joint_name),
      "%s_finger[%d]/joint_base",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerBaseJoints))
      return false;

    // Get the first pair of flex/twist joints
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_flex_from_proximal_to_1",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->flexureFlexJoints[f]))
      return false;
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_twist_from_proximal_to_1",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->flexureTwistJoints[f]))
      return false;

    // Get the sequence of flex/twist joints, one pair at a time.
    for(int l=1; l<(this->numFlexLinks); ++l)
    {
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_flex_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      if(!this->GetAndPushBackJoint(joint_name, 
            this->flexureFlexJoints[f]))
        return false;
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_twist_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      if(!this->GetAndPushBackJoint(joint_name, 
            this->flexureTwistJoints[f]))
        return false;
    }

    // Get the last pair of flex/twist joints
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_flex_from_%d_to_distal",
      this->side.c_str(), f, this->numFlexLinks);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->flexureFlexJoints[f]))
      return false;
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_twist_from_%d_to_distal",
      this->side.c_str(), f, this->numFlexLinks);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->flexureTwistJoints[f]))
      return false;
  }

  gzlog << "IRobotHandPlugin found all joints for " << this->side
        << " hand." << std::endl;

  // resize joint states vector
  int numJoints = this->fingerBaseRotationJoints.size() +
    this->fingerBaseJoints.size();
  for (unsigned int i = 0; i < this->flexureFlexJoints.size(); ++i)
    numJoints += this->flexureFlexJoints[i].size();
  for (unsigned int i = 0; i < this->flexureTwistJoints.size(); ++i)
    numJoints += this->flexureTwistJoints[i].size();
  this->jointStates.name.resize(numJoints);
  this->jointStates.position.resize(numJoints);
  this->jointStates.velocity.resize(numJoints);
  this->jointStates.effort.resize(numJoints);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::SetJointSpringDamper()
{
  // Fake springiness by setting joint limits to 0 and modifying cfm/erp.
  // TODO: implement a generic spring in Gazebo that will work with any
  // physics engine.

  // (this->numFlexLinks + 2) flex joints @ 0.029 in-lbs/deg per
  // iRobot estimates
  const double flexJointKp = 0.187733 * (this->numFlexLinks + 2);
  const double flexJointKd = 0.01;  // wild guess
  const double twistJointKp =
    0.187733 * (this->numFlexLinks + 2) * 2.0;  // wild guess
  const double twistJointKd = 0.01;  // wild guess
  // 0.0031 in-lbs / deg per iRobot estimates
  const double baseJointKp = 0.020068;
  const double baseJointKd = 0.1;  // wild guess
  const double baseRotationJointKp = 0.0;  // no spring there
  const double baseRotationJointKd = 1.0;  // wild guess

  // 0.23 in-lbs preload per iRobot data
  const double baseJointPreloadTorque = 0.0259865;
  // calculate position for preloaded tension
  const double baseJointPreloadJointPosition =
    baseJointPreloadTorque / baseJointKp;

  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   flexJointKp, flexJointKd, this->flexJointCFM, this->flexJointERP);
  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   twistJointKp, twistJointKd, this->twistJointCFM, this->twistJointERP);
  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   baseJointKp, baseJointKd, this->baseJointCFM, this->baseJointERP);

  // Handle the flex/twist joints in the flexible section
  for(std::vector<gazebo::physics::Joint_V>::iterator it = 
      this->flexureFlexJoints.begin();
      it != this->flexureFlexJoints.end();
      ++it)
  {
    for(gazebo::physics::Joint_V::iterator iit = it->begin();
        iit != it->end();
        ++iit)
    {
      // Assume that the joints are ordered flex then twist, in pairs.
      (*iit)->SetStiffnessDamping(0, flexJointKp, flexJointKd);
      // (*iit)->SetAttribute("lo_stop", 0, 0.0);
      // (*iit)->SetAttribute("hi_stop", 0, 0.0);
      // (*iit)->SetAttribute("stop_cfm", 0, this->flexJointCFM);
      // (*iit)->SetAttribute("stop_erp", 0, this->flexJointERP);
    }
  }

  for(std::vector<gazebo::physics::Joint_V>::iterator it = 
      this->flexureTwistJoints.begin();
      it != this->flexureTwistJoints.end();
      ++it)
  {
    for(gazebo::physics::Joint_V::iterator iit = it->begin();
        iit != it->end();
        ++iit)
    {
      (*iit)->SetStiffnessDamping(0, twistJointKp, twistJointKd);
      // (*iit)->SetAttribute("lo_stop", 0, 0.0);
      // (*iit)->SetAttribute("hi_stop", 0, 0.0);
      // (*iit)->SetAttribute("stop_cfm", 0, this->twistJointCFM);
      // (*iit)->SetAttribute("stop_erp", 0, this->twistJointERP);
    }
  }

  // Handle the base joints, which are spring-loaded.
  for(gazebo::physics::Joint_V::iterator it = this->fingerBaseJoints.begin();
      it != this->fingerBaseJoints.end();
      ++it)
  {
    (*it)->SetStiffnessDamping(0, baseJointKp, baseJointKd,
      -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("lo_stop", 0, -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("hi_stop", 0, -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("stop_cfm", 0, this->baseJointCFM);
    // (*it)->SetAttribute("stop_erp", 0, this->baseJointERP);
  }

  // Handle the base rotation joints, which are not spring-loaded.
  for(gazebo::physics::Joint_V::iterator
      it = this->fingerBaseRotationJoints.begin();
      it != this->fingerBaseRotationJoints.end();
      ++it)
  {
    (*it)->SetStiffnessDamping(0, baseRotationJointKp, baseRotationJointKd);
  }
}

////////////////////////////////////////////////////////////////////////////////
void IRobotHandPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(IRobotHandPlugin)
