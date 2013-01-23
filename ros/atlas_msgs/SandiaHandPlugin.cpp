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
 * Desc: Plugin for the SandiaHand robot
 * Author: John Hsu
 * Date: December 2012
 */
#include "SandiaHandPlugin.h"

#include "sensor_msgs/Imu.h"

using std::string;

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
SandiaHandPlugin::SandiaHandPlugin()
{
  this->imuLinkName = "imu_link";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SandiaHandPlugin::~SandiaHandPlugin()
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
void SandiaHandPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  gzerr << "Loading SandiaHandPlugin\n";
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();
  this->sdf = _sdf;
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // initialize imu
  this->lastImuTime = this->world->GetSimTime();

  // get joints
  this->joints.push_back(model->GetJoint("left_f0_j0" )); 
  this->joints.push_back(model->GetJoint("left_f0_j1" )); 
  this->joints.push_back(model->GetJoint("left_f0_j2" )); 
  this->joints.push_back(model->GetJoint("left_f1_j0" )); 
  this->joints.push_back(model->GetJoint("left_f1_j1" )); 
  this->joints.push_back(model->GetJoint("left_f1_j2" )); 
  this->joints.push_back(model->GetJoint("left_f2_j0" )); 
  this->joints.push_back(model->GetJoint("left_f2_j1" )); 
  this->joints.push_back(model->GetJoint("left_f2_j2" )); 
  this->joints.push_back(model->GetJoint("left_f3_j0" )); 
  this->joints.push_back(model->GetJoint("left_f3_j1" )); 
  this->joints.push_back(model->GetJoint("left_f3_j2" )); 
  this->joints.push_back(model->GetJoint("right_f0_j0" )); 
  this->joints.push_back(model->GetJoint("right_f0_j1" )); 
  this->joints.push_back(model->GetJoint("right_f0_j2" )); 
  this->joints.push_back(model->GetJoint("right_f1_j0" )); 
  this->joints.push_back(model->GetJoint("right_f1_j1" )); 
  this->joints.push_back(model->GetJoint("right_f1_j2" )); 
  this->joints.push_back(model->GetJoint("right_f2_j0" )); 
  this->joints.push_back(model->GetJoint("right_f2_j1" )); 
  this->joints.push_back(model->GetJoint("right_f2_j2" )); 
  this->joints.push_back(model->GetJoint("right_f3_j0" )); 
  this->joints.push_back(model->GetJoint("right_f3_j1" )); 
  this->joints.push_back(model->GetJoint("right_f3_j2" )); 

  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    if (!this->joints[i])
    {
      ROS_INFO("sandia hand not present, plugin not loaded");
      return;
    }
  }

  this->errorTerms.resize(this->joints.size());

  this->jointStates.name.resize(this->joints.size());
  this->jointStates.position.resize(this->joints.size());
  this->jointStates.velocity.resize(this->joints.size());
  this->jointStates.effort.resize(this->joints.size());

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

  for(unsigned i = 0; i < this->joints.size(); ++i)
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

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&SandiaHandPlugin::DeferredLoad,this ));
}

// helper function to save some typing
void SandiaHandPlugin::CopyVectorIfValid(const std::vector<double> &from,
                                         std::vector<double> &to,
                                         const unsigned joint_offset)
{
  if (joint_offset != 0 && joint_offset != to.size() / 2)
    return; // get outta here, it's all over
  if (!from.size() || from.size() != to.size() / 2)
    return;
  for (size_t i = 0; i < from.size(); i++)
    to[i + joint_offset] = from[i];
}

////////////////////////////////////////////////////////////////////////////////
// Set Joint Commands
void SandiaHandPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg,
  const unsigned ofs) // ofs = joint offset
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
  gzerr << "Deferred Loading SandiaHandPlugin\n";
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
        snprintf(joint_ns, sizeof(joint_ns), "sandia_hand/gains/%s_f%d_j%d/",
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
  /// brief broadcasts the robot states
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hand/joint_states", 10);

  // ros publication / subscription
  this->pubStatus =
    this->rosNode->advertise<std_msgs::String>("sandia_hand/status", 10);

  // ros topic subscriptions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hand/l_hand/joint_commands", 100,
    boost::bind(&SandiaHandPlugin::SetJointCommands, this, _1, 0),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[0] = this->rosNode->subscribe(jointCommandsSo);
  jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hand/r_hand/joint_commands", 100,
    boost::bind(&SandiaHandPlugin::SetJointCommands, this, _1, 12),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[1] = this->rosNode->subscribe(jointCommandsSo);
 
  // publish imu data
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hand/imu", 10);

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0; // Hz

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind( &SandiaHandPlugin::RosQueueThread,this ) );

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&SandiaHandPlugin::UpdateStates, this));
}

void SandiaHandPlugin::UpdateStates()
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
    }

    // populate FromRobot from robot
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->jointStates.position[i] = this->joints[i]->GetAngle(0).Radian();
      this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
      // better to us e GetForceTorque dot joint axis ??
      this->jointStates.effort[i] = this->joints[i]->GetForce(0);
    }
    this->pubJointStates.publish(this->jointStates);

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    /// update pid with feedforward force
    for(unsigned int i = 0; i < this->joints.size(); ++i)
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
      double force = this->jointCommands.kp_position[i] * this->errorTerms[i].q_p +
                     this->jointCommands.kp_velocity[i] * this->errorTerms[i].qd_p +
                     this->jointCommands.ki_position[i] * this->errorTerms[i].q_i +
                     this->jointCommands.kd_position[i] * this->errorTerms[i].d_q_p_dt +
                     this->jointCommands.effort[i];

      this->joints[i]->SetForce(0, force);

    }

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

GZ_REGISTER_MODEL_PLUGIN(SandiaHandPlugin)
}
