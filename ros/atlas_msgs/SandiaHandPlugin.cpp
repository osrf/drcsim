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
#include <vector>
#include <sensor_msgs/Imu.h>
#include <sandia_hand_msgs/RawTactile.h>

#include "gazebo/transport/Node.hh"
#include "SandiaHandPlugin.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SandiaHandPlugin)



const static double fingerTactileArray[] = {-6.1, 46.0, 1.8,
                                            0.0, 46.0, 3.8,
                                            6.1, 46.0, 1.8,
                                            -6.3, 37.0, 2.1,
                                            0.0, 37.0, 4.2,
                                            6.3, 37.0, 2.1,
                                            -6.5, 28.0, 2.4,
                                            0.0, 28.0, 4.5,
                                            6.5, 28.0, 2.4,
                                            -6.7, 18.9, 2.7,
                                            0.0, 18.9, 4.9,
                                            6.7, 18.9, 2.7,
                                            -7.6, 30.0, 4.0,
                                            0.0, 30.0, 6.5,
                                            7.6, 30.0, 4.0,
                                            -7.9, 20.0, 4.3,
                                            0.0, 20.0, 6.9,
                                            7.9, 20.0, 4.3};

const static double palmTactileArray[] = {-68, 86.9, 35.8,
                                          -36.7, 111.8, 35.8,
                                          -5.5, 136.9, 35.8,
                                          -68.2, 75.2, 36.5,
                                          -56.5, 84.5, 36.5,
                                          -37, 100.2, 36.5,
                                          -25.3, 109.5, 36.5,
                                          -5.8, 125.2, 36.5,
                                          5.9, 134.5, 36.5,
                                          -50.4, 78.5, 37.2,
                                          -39.5, 87.3, 37.2,
                                          -19.2, 103.5, 37.2,
                                          -8.3, 112.3, 37.2,
                                          -44.8, 71.5, 38,
                                          -33.9, 80.3, 38,
                                          -13.6, 96.5, 38,
                                          -2.7, 5.3, 38,
                                          -51.7, 54.5, 38.7,
                                          -36.1, 67, 38.7,
                                          -20.4, 79.5, 38.7,
                                          -4.8, 92,38.7,
                                          10.8, 104.5, 38.7,
                                          -33.5, 57.8, 42.7,
                                          0.1, 84.7, 42.7,
                                          -30.8, 54.5, 50.5,
                                          2.7, 81.4, 50.5,
                                          -26.2, 48.7, 55.2,
                                          7.4, 75.6,55.2,
                                          -20.4, 41.5, 54.9,
                                          13.2, 68.4, 54.9,
                                          -15.4, 35.2, 50.4,
                                          18.2, 62.1, 50.4};

////////////////////////////////////////////////////////////////////////////////
// Constructor
SandiaHandPlugin::SandiaHandPlugin()
{
  this->leftImuLinkName = "l_hand";
  this->rightImuLinkName = "r_hand";

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SandiaHandPlugin::~SandiaHandPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void SandiaHandPlugin::Load(physics::ModelPtr _parent,
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
  this->jointNames.push_back("left_f0_j0");
  this->jointNames.push_back("left_f0_j1");
  this->jointNames.push_back("left_f0_j2");
  this->jointNames.push_back("left_f1_j0");
  this->jointNames.push_back("left_f1_j1");
  this->jointNames.push_back("left_f1_j2");
  this->jointNames.push_back("left_f2_j0");
  this->jointNames.push_back("left_f2_j1");
  this->jointNames.push_back("left_f2_j2");
  this->jointNames.push_back("left_f3_j0");
  this->jointNames.push_back("left_f3_j1");
  this->jointNames.push_back("left_f3_j2");
  this->jointNames.push_back("right_f0_j0");
  this->jointNames.push_back("right_f0_j1");
  this->jointNames.push_back("right_f0_j2");
  this->jointNames.push_back("right_f1_j0");
  this->jointNames.push_back("right_f1_j1");
  this->jointNames.push_back("right_f1_j2");
  this->jointNames.push_back("right_f2_j0");
  this->jointNames.push_back("right_f2_j1");
  this->jointNames.push_back("right_f2_j2");
  this->jointNames.push_back("right_f3_j0");
  this->jointNames.push_back("right_f3_j1");
  this->jointNames.push_back("right_f3_j2");

  this->joints.resize(this->jointNames.size());

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])
    {
      ROS_ERROR("sandia hand not present, plugin not loaded");
      return;
    }
  }

  this->errorTerms.resize(this->joints.size());

  this->leftJointStates.name.resize(this->joints.size() / 2);
  this->leftJointStates.position.resize(this->joints.size() / 2);
  this->leftJointStates.velocity.resize(this->joints.size() / 2);
  this->leftJointStates.effort.resize(this->joints.size() / 2);

  this->rightJointStates.name.resize(this->joints.size() / 2);
  this->rightJointStates.position.resize(this->joints.size() / 2);
  this->rightJointStates.velocity.resize(this->joints.size() / 2);
  this->rightJointStates.effort.resize(this->joints.size() / 2);

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    if (i < this->joints.size() / 2)
    {
      this->leftJointStates.name[i] = this->jointNames[i];
    }
    else
    {
      unsigned j = i - this->joints.size() / 2;
      this->rightJointStates.name[j] = this->jointNames[i];
    }
  }

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
  this->leftImuLink = this->model->GetLink(this->leftImuLinkName);
  if (!this->leftImuLink)
    gzerr << this->leftImuLinkName << " not found\n";
  else
  {
    // initialize imu reference pose
    this->leftImuReferencePose = this->leftImuLink->GetWorldPose();
    this->leftImuLastLinearVel = leftImuReferencePose.rot.RotateVector(
      this->leftImuLink->GetWorldLinearVel());
  }

  this->rightImuLink = this->model->GetLink(this->rightImuLinkName);
  if (!this->rightImuLink)
    gzerr << this->rightImuLinkName << " not found\n";
  else
  {
    // initialize imu reference pose
    this->rightImuReferencePose = this->rightImuLink->GetWorldPose();
    this->rightImuLastLinearVel = rightImuReferencePose.rot.RotateVector(
      this->rightImuLink->GetWorldLinearVel());
  }

  // Contact data
  this->node.reset(new transport::Node());
  this->node->Init(this->world->GetName());

  // FIXME: remove hard coded topic names
  this->contactSub[0] = this->node->Subscribe("~/atlas/contact_0",
      &SandiaHandPlugin::OnRContacts, this);
  this->contactSub[1] = this->node->Subscribe("~/atlas/contact_1",
      &SandiaHandPlugin::OnLContacts, this);


  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&SandiaHandPlugin::DeferredLoad, this));
}

// helper function to save some typing
void SandiaHandPlugin::CopyVectorIfValid(const std::vector<double> &from,
                                         std::vector<double> &to,
                                         const unsigned joint_offset)
{
  if (joint_offset != 0 && joint_offset != to.size() / 2)
    return;  // get outta here, it's all over
  if (!from.size() || from.size() != to.size() / 2)
    return;
  for (size_t i = 0; i < from.size(); i++)
    to[i + joint_offset] = from[i];
}

////////////////////////////////////////////////////////////////////////////////
// Set Joint Commands
void SandiaHandPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg,
  const unsigned ofs)  // ofs = joint offset
{
  // this implementation does not check the ordering of the joints. they must
  // agree with the structure initialized above!
  CopyVectorIfValid(_msg->position, this->jointCommands.position, ofs);
  CopyVectorIfValid(_msg->velocity, this->jointCommands.velocity, ofs);
  CopyVectorIfValid(_msg->effort, this->jointCommands.effort, ofs);
  CopyVectorIfValid(_msg->kp_position, this->jointCommands.kp_position, ofs);
  CopyVectorIfValid(_msg->ki_position, this->jointCommands.ki_position, ofs);
  CopyVectorIfValid(_msg->kd_position, this->jointCommands.kd_position, ofs);
  CopyVectorIfValid(_msg->kp_velocity, this->jointCommands.kp_velocity, ofs);
  CopyVectorIfValid(_msg->i_effort_min, this->jointCommands.i_effort_min, ofs);
  CopyVectorIfValid(_msg->i_effort_max, this->jointCommands.i_effort_max, ofs);
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void SandiaHandPlugin::DeferredLoad()
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

  // publish multi queue
  this->pmq.startServiceThread();

  // pull down controller parameters; they should be on the param server by now
  const int NUM_SIDES = 2, NUM_FINGERS = 4, NUM_FINGER_JOINTS = 3;
  const char *sides[NUM_SIDES] = {"left", "right"};
  for (int side = 0; side < NUM_SIDES; side++)
  {
    for (int finger = 0; finger < NUM_FINGERS; finger++)
    {
      for (int joint = 0; joint < NUM_FINGER_JOINTS; joint++)
      {
        char joint_ns[200] = "";
        snprintf(joint_ns, sizeof(joint_ns), "sandia_hands/gains/%s_f%d_j%d/",
                 sides[side], finger, joint);
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
        int joint_idx = side * (NUM_FINGERS * NUM_FINGER_JOINTS) +
                        finger * NUM_FINGER_JOINTS +
                        joint;
        this->jointCommands.kp_position[joint_idx]  =  p_val;
        this->jointCommands.ki_position[joint_idx]  =  i_val;
        this->jointCommands.kd_position[joint_idx]  =  d_val;
        this->jointCommands.i_effort_min[joint_idx] = -i_clamp_val;
        this->jointCommands.i_effort_max[joint_idx] =  i_clamp_val;
      }
    }
  }

  // ROS Controller API

  // ros publication / subscription
  /// brief broadcasts the robot states
  this->pubLeftJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
  this->pubLeftJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hands/l_hand/joint_states", 10);
  this->pubRightJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
  this->pubRightJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hands/r_hand/joint_states", 10);

  // ros topic subscriptions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hands/l_hand/joint_commands", 100,
    boost::bind(&SandiaHandPlugin::SetJointCommands, this, _1, 0),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[0] = this->rosNode->subscribe(jointCommandsSo);
  jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hands/r_hand/joint_commands", 100,
    boost::bind(&SandiaHandPlugin::SetJointCommands, this, _1, 12),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[1] = this->rosNode->subscribe(jointCommandsSo);

  // publish imu data
  this->pubLeftImuQueue = this->pmq.addPub<sensor_msgs::Imu>();
  this->pubLeftImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/l_hand/imu", 10);
  this->pubRightImuQueue = this->pmq.addPub<sensor_msgs::Imu>();
  this->pubRightImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/r_hand/imu", 10);

  // publish contact data
  this->pubLeftTactileQueue = this->pmq.addPub<sandia_hand_msgs::RawTactile>();
  this->pubLeftTactile =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/l_hand/tactile_raw", 10);
  this->pubRightTactileQueue = this->pmq.addPub<sandia_hand_msgs::RawTactile>();
  this->pubRightTactile =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/r_hand/tactile_raw", 10);

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&SandiaHandPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&SandiaHandPlugin::UpdateStates, this));
}

void SandiaHandPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();
  /// @todo:  robot internals
  /// self diagnostics, damages, etc.

  if (curTime > this->lastControllerUpdateTime)
  {
    // get imu data from imu link
    if (curTime > this->lastImuTime)
    {
      if (this->leftImuLink)
      {
        // Get imuLnk Pose/Orientation
        math::Pose leftImuPose = this->leftImuLink->GetWorldPose();
        math::Vector3 leftImuLinearVel = leftImuPose.rot.RotateVector(
          this->leftImuLink->GetWorldLinearVel());

        sensor_msgs::Imu leftImuMsg;
        leftImuMsg.header.frame_id = this->leftImuLinkName;
        leftImuMsg.header.stamp = ros::Time(curTime.Double());

        // compute angular rates
        {
          // get world twist and convert to local frame
          math::Vector3 wLocal = leftImuPose.rot.RotateVector(
            this->leftImuLink->GetWorldAngularVel());
          leftImuMsg.angular_velocity.x = wLocal.x;
          leftImuMsg.angular_velocity.y = wLocal.y;
          leftImuMsg.angular_velocity.z = wLocal.z;
        }

        // compute acceleration
        {
          math::Vector3 accel = leftImuLinearVel - this->leftImuLastLinearVel;
          double leftImuDdx = accel.x;
          double leftImuDdy = accel.y;
          double leftImuDdz = accel.z;

          leftImuMsg.linear_acceleration.x = leftImuDdx;
          leftImuMsg.linear_acceleration.y = leftImuDdy;
          leftImuMsg.linear_acceleration.z = leftImuDdz;

          this->leftImuLastLinearVel = leftImuLinearVel;
        }

        // compute orientation
        {
          // Get IMU rotation relative to Initial IMU Reference Pose
          math::Quaternion leftImuRot =
            leftImuPose.rot * this->leftImuReferencePose.rot.GetInverse();

          leftImuMsg.orientation.x = leftImuRot.x;
          leftImuMsg.orientation.y = leftImuRot.y;
          leftImuMsg.orientation.z = leftImuRot.z;
          leftImuMsg.orientation.w = leftImuRot.w;
        }

        this->pubLeftImuQueue->push(leftImuMsg, this->pubLeftImu);
      }

      if (this->rightImuLink)
      {
        // Get imuLnk Pose/Orientation
        math::Pose rightImuPose = this->rightImuLink->GetWorldPose();
        math::Vector3 rightImuLinearVel = rightImuPose.rot.RotateVector(
          this->rightImuLink->GetWorldLinearVel());

        sensor_msgs::Imu rightImuMsg;
        rightImuMsg.header.frame_id = this->rightImuLinkName;
        rightImuMsg.header.stamp = ros::Time(curTime.Double());

        // compute angular rates
        {
          // get world twist and convert to local frame
          math::Vector3 wLocal = rightImuPose.rot.RotateVector(
            this->rightImuLink->GetWorldAngularVel());
          rightImuMsg.angular_velocity.x = wLocal.x;
          rightImuMsg.angular_velocity.y = wLocal.y;
          rightImuMsg.angular_velocity.z = wLocal.z;
        }

        // compute acceleration
        {
          math::Vector3 accel = rightImuLinearVel - this->rightImuLastLinearVel;
          double rightImuDdx = accel.x;
          double rightImuDdy = accel.y;
          double rightImuDdz = accel.z;

          rightImuMsg.linear_acceleration.x = rightImuDdx;
          rightImuMsg.linear_acceleration.y = rightImuDdy;
          rightImuMsg.linear_acceleration.z = rightImuDdz;

          this->rightImuLastLinearVel = rightImuLinearVel;
        }

        // compute orientation
        {
          // Get IMU rotation relative to Initial IMU Reference Pose
          math::Quaternion rightImuRot =
            rightImuPose.rot * this->rightImuReferencePose.rot.GetInverse();

          rightImuMsg.orientation.x = rightImuRot.x;
          rightImuMsg.orientation.y = rightImuRot.y;
          rightImuMsg.orientation.z = rightImuRot.z;
          rightImuMsg.orientation.w = rightImuRot.w;
        }

        this->pubRightImuQueue->push(rightImuMsg, this->pubRightImu);
      }

      // update time
      this->lastImuTime = curTime.Double();
    }

    // populate FromRobot from robot
    this->leftJointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->rightJointStates.header.stamp = this->leftJointStates.header.stamp;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      // The following is added to fix compiler warnings.
      unsigned int i0 = 0;
      if (i < this->joints.size() / 2)
      {
        this->leftJointStates.position[i] =
          this->joints[i]->GetAngle(0).Radian();
        this->leftJointStates.velocity[i] = this->joints[i]->GetVelocity(0);
        // better to use GetForceTorque dot joint axis
        this->leftJointStates.effort[i] = this->joints[i]->GetForce(i0);
      }
      else
      {
        unsigned j = i - this->joints.size() / 2;
        this->rightJointStates.position[j] =
          this->joints[i]->GetAngle(0).Radian();
        this->rightJointStates.velocity[j] = this->joints[i]->GetVelocity(0);
        this->rightJointStates.effort[j] = this->joints[i]->GetForce(i0);
      }
    }
    this->pubLeftJointStatesQueue->push(this->leftJointStates,
      this->pubLeftJointStates);
    this->pubRightJointStatesQueue->push(this->rightJointStates,
      this->pubRightJointStates);

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    /// update pid with feedforward force
    double position;
    double velocity;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      if (i < this->joints.size() / 2)
      {
        position = this->leftJointStates.position[i];
        velocity = this->leftJointStates.velocity[i];
      }
      else
      {
        unsigned j = i - this->joints.size() / 2;
        position = this->rightJointStates.position[j];
        velocity = this->rightJointStates.velocity[j];
      }

      double q_p =
         this->jointCommands.position[i] - position;

      if (!math::equal(dt, 0.0))
        this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

      this->errorTerms[i].q_p = q_p;

      this->errorTerms[i].qd_p =
         this->jointCommands.velocity[i] - velocity;

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


    // publish tactile data
    {
      boost::mutex::scoped_lock lock(this->contactRMutex);

      sandia_hand_msgs::RawTactile leftTactileMsg;
      std::vector<std::string>::iterator collIter;
      std::string collision1;

      // Don't do anything if there is no new data to process.
      if (!this->incomingRContacts.empty())
      {
        // Iterate over all the contact messages
        for (ContactMsgs_L::iterator iter = this->incomingRContacts.begin();
            iter != this->incomingRContacts.end(); ++iter)
        {
          // Iterate over all the contacts in the message
          for (int i = 0; i < (*iter)->contact_size(); ++i)
          {
            // Get the collision pointer from name in contact msg
            collision1 = (*iter)->contact(i).collision1();
            if (collision1.find("right_f") ==  string::npos
                && collision1.find("palm") ==  string::npos)
              collision1 = (*iter)->contact(i).collision2();

//            gzerr << " right got col  " << (*iter)->contact(i).collision1()
//                << " " << (*iter)->contact(i).collision2() << std::endl;

            physics::Collision *col = NULL;
            if (!this->contactCollisions.count(collision1))
            {
              col = boost::dynamic_pointer_cast<physics::Collision>(
                  this->world->GetEntity(collision1)).get();
              this->contactCollisions[collision1] = col;
            }
            else
            {
              col = this->contactCollisions[collision1];
            }
//            gzerr << " right got col  " << collision1 << std::endl;

            GZ_ASSERT(col, "Contact collision is Null!");

            math::Vector3 pos;
            math::Vector3 normal;
            // Iterate all contact positions
            for (int j = 0; j < (*iter)->contact(i).position_size(); ++j)
            {
              pos = msgs::Convert((*iter)->contact(i).position(j));
              normal = msgs::Convert((*iter)->contact(i).normal(j));

              // Use collision pose to transform contact msg position into
              // local frame
              // pos = col->GetWorldPose().rot.RotateVectorReverse(pos);
              // normal = col->GetWorldPose().rot.RotateVectorReverse(normal);

              // gzerr << " contact pos for " << collision1 << ": " << pos
              //  << " world pose col " << col->GetWorldPose() << std::endl;

              // check against tactile array positions

              // Set it to on if it's within bounds
            }
          }
        }
      }
      // Clear the incoming contact list.
      this->incomingRContacts.clear();
      this->pubLeftTactileQueue->push(leftTactileMsg, this->pubLeftTactile);
    }

    sandia_hand_msgs::RawTactile rightTactileMsg;
    this->pubRightTactileQueue->push(rightTactileMsg, this->pubRightTactile);
    this->lastControllerUpdateTime = curTime;
  }
}

void SandiaHandPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

//////////////////////////////////////////////////
void SandiaHandPlugin::OnRContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->contactRMutex);

  // Only store information if the model is active
  // if (this->IsActive())
  {
    // Store the contacts message for processing in UpdateImpl
    this->incomingRContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->incomingRContacts.size() > 100)
      this->incomingRContacts.pop_front();
  }
}

//////////////////////////////////////////////////
void SandiaHandPlugin::OnLContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->contactLMutex);

  // Only store information if the model is active
  // if (this->IsActive())
  {
    // Store the contacts message for processing in UpdateImpl
    this->incomingLContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->incomingLContacts.size() > 100)
      this->incomingLContacts.pop_front();
  }
}

}
