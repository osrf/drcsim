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

#include <gazebo/transport/Node.hh>
#include "drcsim_gazebo_ros_plugins/SandiaHandPlugin.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SandiaHandPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
SandiaHandPlugin::SandiaHandPlugin()
{
  this->leftImuLinkName = "l_hand";
  this->rightImuLinkName = "r_hand";
  this->hasStumps = false;
  this->leftTactileConnectCount = 0;
  this->rightTactileConnectCount = 0;
  this->pmq = new PubMultiQueue();
  this->rosNode = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SandiaHandPlugin::~SandiaHandPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  delete this->pmq;
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

  // Get hand joints
  {
    unsigned int jointCount = 0;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->joints[i] = this->model->GetJoint(this->jointNames[i]);
      if (this->joints[i])
        ++jointCount;
    }
    if (jointCount == 0)
    {
      this->hasStumps = true;
      ROS_INFO("No sandia hand joints found, load as stumps");
    }
    else if (jointCount != this->joints.size())
    {
      ROS_ERROR("Error loading sandia hand joints, plugin not loaded");
      return;
    }
  }

  {
    // kp_velocity bounds Nms/rad
    this->jointDampingMax.push_back(30.0);  // left_f0_j0
    this->jointDampingMax.push_back(30.0);  // left_f0_j1
    this->jointDampingMax.push_back(30.0);  // left_f0_j2
    this->jointDampingMax.push_back(30.0);  // left_f1_j0
    this->jointDampingMax.push_back(30.0);  // left_f1_j1
    this->jointDampingMax.push_back(30.0);  // left_f1_j2
    this->jointDampingMax.push_back(30.0);  // left_f2_j0
    this->jointDampingMax.push_back(30.0);  // left_f2_j1
    this->jointDampingMax.push_back(30.0);  // left_f2_j2
    this->jointDampingMax.push_back(30.0);  // left_f3_j0
    this->jointDampingMax.push_back(30.0);  // left_f3_j1
    this->jointDampingMax.push_back(30.0);  // left_f3_j2
    this->jointDampingMax.push_back(30.0);  // right_f0_j0
    this->jointDampingMax.push_back(30.0);  // right_f0_j1
    this->jointDampingMax.push_back(30.0);  // right_f0_j2
    this->jointDampingMax.push_back(30.0);  // right_f1_j0
    this->jointDampingMax.push_back(30.0);  // right_f1_j1
    this->jointDampingMax.push_back(30.0);  // right_f1_j2
    this->jointDampingMax.push_back(30.0);  // right_f2_j0
    this->jointDampingMax.push_back(30.0);  // right_f2_j1
    this->jointDampingMax.push_back(30.0);  // right_f2_j2
    this->jointDampingMax.push_back(30.0);  // right_f3_j0
    this->jointDampingMax.push_back(30.0);  // right_f3_j1
    this->jointDampingMax.push_back(30.0);  // right_f3_j2

    this->jointDampingMin.push_back(0);  // left_f0_j0
    this->jointDampingMin.push_back(0);  // left_f0_j1
    this->jointDampingMin.push_back(0);  // left_f0_j2
    this->jointDampingMin.push_back(0);  // left_f1_j0
    this->jointDampingMin.push_back(0);  // left_f1_j1
    this->jointDampingMin.push_back(0);  // left_f1_j2
    this->jointDampingMin.push_back(0);  // left_f2_j0
    this->jointDampingMin.push_back(0);  // left_f2_j1
    this->jointDampingMin.push_back(0);  // left_f2_j2
    this->jointDampingMin.push_back(0);  // left_f3_j0
    this->jointDampingMin.push_back(0);  // left_f3_j1
    this->jointDampingMin.push_back(0);  // left_f3_j2
    this->jointDampingMin.push_back(0);  // right_f0_j0
    this->jointDampingMin.push_back(0);  // right_f0_j1
    this->jointDampingMin.push_back(0);  // right_f0_j2
    this->jointDampingMin.push_back(0);  // right_f1_j0
    this->jointDampingMin.push_back(0);  // right_f1_j1
    this->jointDampingMin.push_back(0);  // right_f1_j2
    this->jointDampingMin.push_back(0);  // right_f2_j0
    this->jointDampingMin.push_back(0);  // right_f2_j1
    this->jointDampingMin.push_back(0);  // right_f2_j2
    this->jointDampingMin.push_back(0);  // right_f3_j0
    this->jointDampingMin.push_back(0);  // right_f3_j1
    this->jointDampingMin.push_back(0);  // right_f3_j2
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
      this->leftJointStates.position[i] = 0;
      this->leftJointStates.velocity[i] = 0;
      this->leftJointStates.effort[i] = 0;
    }
    else
    {
      unsigned j = i - this->joints.size() / 2;
      this->rightJointStates.name[j] = this->jointNames[i];
      this->rightJointStates.position[j] = 0;
      this->rightJointStates.velocity[j] = 0;
      this->rightJointStates.effort[j] = 0;
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
    if (!this->hasStumps)
      this->jointCommands.name[i] = this->joints[i]->GetScopedName();
    else
      this->jointCommands.name[i] = this->jointNames[i];
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

  this->rightImuLink = this->model->GetLink(this->rightImuLinkName);
  if (!this->rightImuLink)
    gzerr << this->rightImuLinkName << " not found\n";

  // Get imu sensors
  this->leftImuSensor =
    boost::shared_dynamic_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->leftImuLink->GetScopedName()
        + "::imu_sensor"));
  if (!this->leftImuSensor)
    gzerr << "left imu_sensor not found\n" << "\n";

  this->rightImuSensor =
    boost::shared_dynamic_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->rightImuLink->GetScopedName()
        + "::imu_sensor"));
  if (!this->rightImuSensor)
    gzerr << "right imu_sensor not found\n" << "\n";


  // Tactile data
  this->tactileFingerArraySize = 18;
  this->tactilePalmArraySize = 32;

  // Approximate output range of the tactile sensor
  // determined by experimenting with the actual physical hand
  this->maxTactileOut = 33500;
  this->minTactileOut = 26500;

  this->leftTactile.f0.resize(this->tactileFingerArraySize);
  this->leftTactile.f1.resize(this->tactileFingerArraySize);
  this->leftTactile.f2.resize(this->tactileFingerArraySize);
  this->leftTactile.f3.resize(this->tactileFingerArraySize);
  this->rightTactile.f0.resize(this->tactileFingerArraySize);
  this->rightTactile.f1.resize(this->tactileFingerArraySize);
  this->rightTactile.f2.resize(this->tactileFingerArraySize);
  this->rightTactile.f3.resize(this->tactileFingerArraySize);
  this->leftTactile.palm.resize(this->tactilePalmArraySize);
  this->rightTactile.palm.resize(this->tactilePalmArraySize);
  for (int i = 0; i < this->tactileFingerArraySize; ++i)
  {
    this->leftTactile.f0[i] = this->minTactileOut;
    this->leftTactile.f1[i] = this->minTactileOut;
    this->leftTactile.f2[i] = this->minTactileOut;
    this->leftTactile.f3[i] = this->minTactileOut;
    this->rightTactile.f0[i] = this->minTactileOut;
    this->rightTactile.f1[i] = this->minTactileOut;
    this->rightTactile.f2[i] = this->minTactileOut;
    this->rightTactile.f3[i] = this->minTactileOut;
  }
  for (int i = 0; i < this->tactilePalmArraySize; ++i)
  {
    this->leftTactile.palm[i] = this->minTactileOut;
    this->rightTactile.palm[i] = this->minTactileOut;
  }

  if (!hasStumps)
  {
    // Sandia hand tactile dimensions taken from spec and adapted to fit on our
    // sandia hand model
    this->palmColWidth[0] = 0.01495;
    this->palmColLength[0] = 0.02341;
    this->palmColWidth[1] = 0.01495;
    this->palmColLength[1] = 0.02341;
    this->palmColWidth[2] = 0.01495;
    this->palmColLength[2] = 0.02341;
    this->palmColWidth[3] = 0.04304;
    this->palmColLength[3] = 0.05271;
    this->palmColWidth[4] = 0.08004;
    this->palmColLength[4] = 0.01170;

    this->palmHorSize[0] = 2;
    this->palmVerSize[0] = 3;
    this->palmHorSize[1] = 2;
    this->palmVerSize[1] = 3;
    this->palmHorSize[2] = 2;
    this->palmVerSize[2] = 3;
    this->palmHorSize[3] = 2;
    this->palmVerSize[3] = 5;
    this->palmHorSize[4] = 5;
    this->palmVerSize[4] = 2;

    this->fingerColLength[0] = 0.01;
    this->fingerColWidth[0] = 0.0158;
    this->fingerColLength[1] = 0.0271;
    this->fingerColWidth[1] = 0.0134;

    this->fingerHorSize[0] = 3;
    this->fingerVerSize[0] = 2;
    this->fingerHorSize[1] = 3;
    this->fingerVerSize[1] = 4;

    this->node.reset(new transport::Node());
    this->node->Init(this->world->GetName());

    std::string modelName = this->model->GetName();
    this->contactSub[0] =
        this->node->Subscribe("~/" + modelName + "/contact_0",
        &SandiaHandPlugin::OnRContacts, this);
    this->contactSub[1] =
        this->node->Subscribe("~/" + modelName + "/contact_1",
        &SandiaHandPlugin::OnLContacts, this);
  }

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
  boost::mutex::scoped_lock lock(this->mutex);
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
  this->pmq->startServiceThread();

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
  this->pubLeftJointStatesQueue = this->pmq->addPub<sensor_msgs::JointState>();
  this->pubLeftJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hands/l_hand/joint_states", 10);


  this->pubRightJointStatesQueue = this->pmq->addPub<sensor_msgs::JointState>();
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
  this->pubLeftImuQueue = this->pmq->addPub<sensor_msgs::Imu>();
  this->pubLeftImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/l_hand/imu", 10);
  this->pubRightImuQueue = this->pmq->addPub<sensor_msgs::Imu>();
  this->pubRightImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/r_hand/imu", 10);

  // publish contact data
  this->pubLeftTactileQueue = this->pmq->addPub<sandia_hand_msgs::RawTactile>();
  this->pubLeftTactile =
    this->rosNode->advertise<sandia_hand_msgs::RawTactile>(
      "sandia_hands/l_hand/tactile_raw", 10,
    boost::bind(&SandiaHandPlugin::LeftTactileConnect, this),
    boost::bind(&SandiaHandPlugin::LeftTactileDisconnect, this));

  this->pubRightTactileQueue = this->pmq->addPub<sandia_hand_msgs::RawTactile>();
  this->pubRightTactile =
    this->rosNode->advertise<sandia_hand_msgs::RawTactile>(
      "sandia_hands/r_hand/tactile_raw", 10,
    boost::bind(&SandiaHandPlugin::RightTactileConnect, this),
    boost::bind(&SandiaHandPlugin::RightTactileDisconnect, this));

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&SandiaHandPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&SandiaHandPlugin::UpdateStates, this));

  // Offer teams ability to change damping coef. between preset bounds
  ros::AdvertiseServiceOptions setJointDampingAso =
    ros::AdvertiseServiceOptions::create<atlas_msgs::SetJointDamping>(
      "sandia_hands/set_joint_damping", boost::bind(
        &SandiaHandPlugin::SetJointDamping, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);
  this->setJointDampingService = this->rosNode->advertiseService(
    setJointDampingAso);

  // Offer teams ability to get damping coef.
  ros::AdvertiseServiceOptions getJointDampingAso =
    ros::AdvertiseServiceOptions::create<atlas_msgs::GetJointDamping>(
      "sandia_hands/get_joint_damping", boost::bind(
        &SandiaHandPlugin::GetJointDamping, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);
  this->getJointDampingService = this->rosNode->advertiseService(
    getJointDampingAso);
}

////////////////////////////////////////////////////////////////////////////////
bool SandiaHandPlugin::SetJointDamping(
  atlas_msgs::SetJointDamping::Request &_req,
  atlas_msgs::SetJointDamping::Response &_res)
{
  _res.success = true;
  std::stringstream statusStream;
  {
    boost::mutex::scoped_lock lock(this->mutex);

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      double d = math::clamp(_req.damping_coefficients[i],
       this->jointDampingMin[i], this->jointDampingMax[i]);
      this->joints[i]->SetDamping(0, d);
      if (!math::equal(d, _req.damping_coefficients[i]))
      {
        statusStream << "requested joint damping for joint ["
                     << this->jointNames[i] << "] of ["
                     << _req.damping_coefficients[i] << "] is "
                     << "truncated to [" << d << "].\n";
        _res.success = false;
      }
    }
  }
  ROS_WARN("%s", statusStream.str().c_str());
  _res.status_message = statusStream.str();

  return _res.success;
}

////////////////////////////////////////////////////////////////////////////////
bool SandiaHandPlugin::GetJointDamping(
  atlas_msgs::GetJointDamping::Request &_req,
  atlas_msgs::GetJointDamping::Response &_res)
{
  _res.success = true;
  _res.status_message = "success";

  {
    boost::mutex::scoped_lock lock(this->mutex);

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      _res.damping_coefficients[i] = this->joints[i]->GetDamping(0);
      _res.damping_coefficients_max[i] = this->jointDampingMax[i];
      _res.damping_coefficients_min[i] = this->jointDampingMin[i];
    }
  }

  return _res.success;
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
      if (this->leftImuSensor)
      {
        math::Vector3 angularVel = this->leftImuSensor->GetAngularVelocity();
        math::Vector3 linearAcc = this->leftImuSensor->GetLinearAcceleration();
        math::Quaternion orientation = this->leftImuSensor->GetOrientation();

        sensor_msgs::Imu leftImuMsg;
        leftImuMsg.header.frame_id = this->leftImuLinkName;
        leftImuMsg.header.stamp = ros::Time(curTime.sec, curTime.nsec);

        leftImuMsg.angular_velocity.x = angularVel.x;
        leftImuMsg.angular_velocity.y = angularVel.y;
        leftImuMsg.angular_velocity.z = angularVel.z;

        leftImuMsg.linear_acceleration.x = linearAcc.x;
        leftImuMsg.linear_acceleration.y = linearAcc.y;
        leftImuMsg.linear_acceleration.z = linearAcc.z;

        leftImuMsg.orientation.x = orientation.x;
        leftImuMsg.orientation.y = orientation.y;
        leftImuMsg.orientation.z = orientation.z;
        leftImuMsg.orientation.w = orientation.w;

        this->pubLeftImuQueue->push(leftImuMsg, this->pubLeftImu);
      }

      if (this->rightImuSensor)
      {
        math::Vector3 angularVel = this->rightImuSensor->GetAngularVelocity();
        math::Vector3 linearAcc = this->rightImuSensor->GetLinearAcceleration();
        math::Quaternion orientation = this->rightImuSensor->GetOrientation();

        sensor_msgs::Imu rightImuMsg;
        rightImuMsg.header.frame_id = this->rightImuLinkName;
        rightImuMsg.header.stamp = ros::Time(curTime.sec, curTime.nsec);

        rightImuMsg.angular_velocity.x = angularVel.x;
        rightImuMsg.angular_velocity.y = angularVel.y;
        rightImuMsg.angular_velocity.z = angularVel.z;

        rightImuMsg.linear_acceleration.x = linearAcc.x;
        rightImuMsg.linear_acceleration.y = linearAcc.y;
        rightImuMsg.linear_acceleration.z = linearAcc.z;

        rightImuMsg.orientation.x = orientation.x;
        rightImuMsg.orientation.y = orientation.y;
        rightImuMsg.orientation.z = orientation.z;
        rightImuMsg.orientation.w = orientation.w;

        this->pubRightImuQueue->push(rightImuMsg, this->pubRightImu);
      }

      // update time
      this->lastImuTime = curTime.Double();
    }

    // populate FromRobot from robot
    this->leftJointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->rightJointStates.header.stamp = this->leftJointStates.header.stamp;
    if (!this->hasStumps)
    {
      for (unsigned int i = 0; i < this->joints.size(); ++i)
      {
        if (i < this->joints.size() / 2)
        {
          this->leftJointStates.position[i] =
            this->joints[i]->GetAngle(0).Radian();
          this->leftJointStates.velocity[i] = this->joints[i]->GetVelocity(0);
          // better to use GetForceTorque dot joint axis
          this->leftJointStates.effort[i] = this->joints[i]->GetForce(0u);
        }
        else
        {
          unsigned j = i - this->joints.size() / 2;
          this->rightJointStates.position[j] =
            this->joints[i]->GetAngle(0).Radian();
          this->rightJointStates.velocity[j] = this->joints[i]->GetVelocity(0);
          this->rightJointStates.effort[j] = this->joints[i]->GetForce(0u);
        }
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

      double force;
      {
        boost::mutex::scoped_lock lock(this->mutex);
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
        force = this->jointCommands.kp_position[i] *
                this->errorTerms[i].q_p +
                this->jointCommands.kp_velocity[i] *
                this->errorTerms[i].qd_p +
                this->jointCommands.ki_position[i] *
                this->errorTerms[i].q_i +
                this->jointCommands.kd_position[i] *
                this->errorTerms[i].d_q_p_dt +
                this->jointCommands.effort[i];
      }

      if (!this->hasStumps)
        this->joints[i]->SetForce(0, force);
    }

    // publish tactile data
    if (this->rightTactileConnectCount > 0)
    {
      if (!this->hasStumps)
      {
        // first clear all previous tactile data
        for (int i = 0; i < this->tactileFingerArraySize; ++i)
        {
          this->rightTactile.f0[i] = this->minTactileOut;
          this->rightTactile.f1[i] = this->minTactileOut;
          this->rightTactile.f2[i] = this->minTactileOut;
          this->rightTactile.f3[i] = this->minTactileOut;
        }

        for (int i = 0; i < this->tactilePalmArraySize; ++i)
          this->rightTactile.palm[i] = this->minTactileOut;

        // Generate data and publish
        {
          boost::mutex::scoped_lock lock(this->contactRMutex);
          this->rightTactile.header.stamp =
              ros::Time(curTime.sec, curTime.nsec);
          this->FillTactileData(RIGHT_HAND, this->incomingRContacts,
              &this->rightTactile);
          // Clear the incoming contact list.
          this->incomingRContacts.clear();
        }
      }
      this->pubRightTactileQueue->push(this->rightTactile,
          this->pubRightTactile);
    }
    else if (!this->hasStumps)
    {
      boost::mutex::scoped_lock lock(this->contactRMutex);
      this->incomingRContacts.clear();
    }

    if (this->leftTactileConnectCount > 0)
    {
      if (!this->hasStumps)
      {
        // first clear all previous tactile data
        for (int i = 0; i < this->tactileFingerArraySize; ++i)
        {
          this->leftTactile.f0[i] = this->minTactileOut;
          this->leftTactile.f1[i] = this->minTactileOut;
          this->leftTactile.f2[i] = this->minTactileOut;
          this->leftTactile.f3[i] = this->minTactileOut;
        }

        for (int i = 0; i < this->tactilePalmArraySize; ++i)
          this->leftTactile.palm[i] = this->minTactileOut;

        {
          boost::mutex::scoped_lock lock(this->contactLMutex);
          this->leftTactile.header.stamp = ros::Time(curTime.sec, curTime.nsec);
          this->FillTactileData(LEFT_HAND, this->incomingLContacts,
              &this->leftTactile);
          this->incomingLContacts.clear();
        }
      }
      this->pubLeftTactileQueue->push(this->leftTactile,
          this->pubLeftTactile);
    }
    else if (!this->hasStumps)
    {
      boost::mutex::scoped_lock lock(this->contactLMutex);
      this->incomingLContacts.clear();
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

//////////////////////////////////////////////////
void SandiaHandPlugin::OnRContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->contactRMutex);

  // Store the contacts message for processing in UpdateImpl
  this->incomingRContacts.push_back(_msg);

  // Prevent the incomingContacts list to grow indefinitely.
  if (this->incomingRContacts.size() > 50)
    this->incomingRContacts.pop_front();
}

//////////////////////////////////////////////////
void SandiaHandPlugin::OnLContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->contactLMutex);

  // Store the contacts message for processing in UpdateImpl
  this->incomingLContacts.push_back(_msg);

  // Prevent the incomingContacts list to grow indefinitely.
  if (this->incomingLContacts.size() > 50)
    this->incomingLContacts.pop_front();
}

//////////////////////////////////////////////////
void SandiaHandPlugin::FillTactileData(HandEnum _side,
    ContactMsgs_L _incomingContacts,
    sandia_hand_msgs::RawTactile *_tactileMsg)
{
  // The method of generating tactile sensor output is specific
  // to the current sandia hand collisions. This is because the collisions
  // do not really match the actual sandia hand so it was not possible to
  // directly use the position of the sensors from the spec sheet. The best
  // we could do is approximate the locations of tactile sensors
  // on these collision. Idea as follows:
  // Divide each collision into smaller regions,
  // Identify the region which the contact point lies,
  // Set the corresponding (closest) tactile sensor's output value.

  // Don't do anything if there is no new data to process.
  if (!_incomingContacts.empty())
  {
    std::vector<std::string>::iterator collIter;
    std::string collision1;

    std::string sideStr = (_side == LEFT_HAND) ? "left" : "right";
    // Iterate over all the contact messages
    for (ContactMsgs_L::iterator iter = _incomingContacts.begin();
        iter != _incomingContacts.end(); ++iter)
    {
      // Iterate over all the contacts in the message
      for (int i = 0; i < (*iter)->contact_size(); ++i)
      {
        bool isPalm = false;
        bool isBody1 = true;
        // Get the collision pointer from name in contact msg
        collision1 = (*iter)->contact(i).collision1();

        if (collision1.find(sideStr + "_f") ==  std::string::npos
            && collision1.find("palm") ==  std::string::npos)
        {
          collision1 = (*iter)->contact(i).collision2();
          isBody1 = false;
        }

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

        GZ_ASSERT(col, "Contact collision is Null!");

        // check if it's a palm or a finger
        if (collision1.find("palm") !=  std::string::npos)
          isPalm = true;

        // get finger index if not palm
        int fingerIdx = -1;
        int fingerColIdx = -1;
        int palmIdx = -1;

        if (isPalm)
        {
          // index finder palm
          if (collision1.find("_3") !=  std::string::npos)
            palmIdx = 0;
          // middle finder palm
          else if (collision1.find("_4") !=  std::string::npos)
            palmIdx = 1;
          // pinky palm
          else if (collision1.find("_5") !=  std::string::npos)
            palmIdx = 2;
          // bottom palm
          else if (collision1.find("_1") !=  std::string::npos)
            palmIdx = 3;
          // mid palm
          else
            palmIdx = 4;
        }
        else
        {
          // index finder
          if (collision1.find("f0") !=  std::string::npos)
            fingerIdx = 0;
          // middle finder
          else if (collision1.find("f1") !=  std::string::npos)
            fingerIdx = 1;
          // pinky
          else if (collision1.find("f2") !=  std::string::npos)
            fingerIdx = 2;
          // thumb
          else if (collision1.find("f3") !=  std::string::npos)
            fingerIdx = 3;

          // lower collision of the finger
          if (collision1.find("_1") !=  std::string::npos)
            fingerColIdx = 0;
          // upper collision of the finger
          else if (collision1.find("_2") !=  std::string::npos)
            fingerColIdx = 1;
        }

        math::Vector3 pos;
        math::Vector3 force;
        int tactileOuput = 0;
        // Iterate all contact positions
        for (int j = 0; j < (*iter)->contact(i).position_size(); ++j)
        {
          pos = msgs::Convert((*iter)->contact(i).position(j));
          if (isBody1)
          {
            force = msgs::Convert((*iter)->contact(i).wrench(j).
                body_1_wrench().force());
          }
          else
          {
            force = msgs::Convert((*iter)->contact(i).wrench(j).
                body_2_wrench().force());
          }

          // Scaling formula taken from Gazebo's ContactVisual class
          tactileOuput = (2.0 * (this->maxTactileOut - this->minTactileOut))
              / (1 + exp(-force.GetSquaredLength() / 100)) -
              (this->maxTactileOut - 2*this->minTactileOut);

          // transfrom into collision frame
          math::Pose colPose = col->GetInitialRelativePose() +
              col->GetLink()->GetWorldPose();
          pos = colPose.rot.GetInverse() * (pos - colPose.pos);

          double vPosInCol = 0;
          double hPosInCol = 0;
          // column
          int ai = 0;
          // row
          int aj = 0;
          // tactile sensor index
          int aIndex = -1;

          // if palm
          if (isPalm)
          {
            switch (palmIdx)
            {
              case 0:
              {
                // Index finger palm sensors: 3; 8 9; 13
                // (numbers correspond to taxel sensor number in spec)
                if (pos.z > 0)
                {
                  vPosInCol =
                      math::clamp(pos.x / this->palmColLength[palmIdx],
                      0.0, 1.0);
                  hPosInCol =
                      math::clamp((-pos.y + this->palmColWidth[palmIdx]/2.0)
                      / this->palmColWidth[0], 0.0, 1.0);

                  ai = this->palmVerSize[palmIdx] -
                      std::ceil(vPosInCol * this->palmVerSize[palmIdx]) - 1;
                  aj = std::ceil(hPosInCol * this->palmHorSize[palmIdx]) - 1;
                  ai = std::max(ai, 0);
                  aj = std::max(aj, 0);
                  aIndex = 2;
                  if (ai == 1)
                  {
                    if (aj == 0)
                      aIndex =7;
                    else
                      aIndex = 8;
                  }
                  else if (ai == 2)
                    aIndex = 12;
                  _tactileMsg->palm[aIndex] = tactileOuput;
                }
                break;
              }
              case 1:
              {
                // Middle finger palm sensors: 2; 6 7; 11 12
                // (numbers correspond to taxel sensor number in spec)
                if (pos.z > 0)
                {
                  // distance apart = w 14.95, h 11.70
                  vPosInCol =
                      math::clamp(pos.x / this->palmColLength[palmIdx],
                      0.0, 1.0);
                  hPosInCol =
                    math::clamp((-pos.y + this->palmColWidth[palmIdx]/2.0)
                    / this->palmColWidth[palmIdx], 0.0, 1.0);
                  ai = this->palmVerSize[palmIdx] -
                      std::ceil(vPosInCol * this->palmVerSize[palmIdx]) - 1;
                  aj = std::ceil(hPosInCol * this->palmHorSize[palmIdx]) - 1;
                  ai = std::max(ai, 0);
                  aj = std::max(aj, 0);
                  aIndex = 1;
                  if (ai == 1)
                  {
                    if (aj == 0)
                      aIndex =5;
                    else
                      aIndex = 6;
                  }
                  else if (ai == 2)
                  {
                    if (aj == 0)
                      aIndex =10;
                    else
                      aIndex = 11;
                  }
                  _tactileMsg->palm[aIndex] = tactileOuput;
                }
                break;
              }
              case 2:
              {
                // Pinky palm sensors: 1; 4 5; 10
                // (numbers correspond to taxel sensor number in spec)
                if (pos.z > 0)
                {
                  vPosInCol =
                      math::clamp(pos.x / this->palmColLength[palmIdx],
                      0.0, 1.0);
                  hPosInCol =
                    math::clamp((-pos.y + this->palmColWidth[palmIdx]/2.0) /
                    this->palmColWidth[palmIdx], 0.0, 1.0);

                  ai = this->palmVerSize[palmIdx] -
                      std::ceil(vPosInCol * this->palmVerSize[palmIdx]) - 1;
                  aj = std::ceil(hPosInCol * this->palmHorSize[palmIdx]) - 1;
                  ai = std::max(ai, 0);
                  aj = std::max(aj, 0);
                  aIndex = 0;
                  if (ai == 1)
                  {
                    if (aj == 0)
                      aIndex =3;
                    else
                      aIndex = 4;
                  }
                  else if (ai == 2)
                    aIndex = 9;
                  _tactileMsg->palm[aIndex] = tactileOuput;
                }
                break;
              }
              case 3:
              {
                // Sensors on bottom palm: 23 24; 25 26; 27 28; 29 30; 31 32
                // (numbers correspond to taxel sensor number in spec)
                int baseIndex = 22;
                if (pos.z > 0)
                {
                  vPosInCol =
                      math::clamp((pos.y + this->palmColLength[palmIdx]/2.0) /
                      this->palmColLength[palmIdx], 0.0, 1.0);
                  hPosInCol =
                      math::clamp((pos.x + this->palmColWidth[palmIdx]/2.0) /
                      this->palmColWidth[palmIdx], 0.0, 1.0);

                  ai = this->palmVerSize[palmIdx] -
                      std::ceil(vPosInCol * this->palmVerSize[palmIdx]) - 1;
                  aj = std::ceil(hPosInCol * this->palmHorSize[palmIdx]) - 1;
                  ai = std::max(ai, 0);
                  aj = std::max(aj, 0);
                  aIndex = baseIndex + ai * this->palmHorSize[palmIdx] + aj;
                  _tactileMsg->palm[aIndex] = tactileOuput;
                }
                break;
              }
              default:
              {
                // Sensors on mid palm (default): 14 15 16 17; 18 19 20 21 22
                // (numbers correspond to taxel sensor number in spec)
                vPosInCol =
                    math::clamp(pos.y / this->palmColLength[palmIdx], 0.0, 1.0);
                hPosInCol =
                    math::clamp((pos.z + this->palmColWidth[palmIdx]/2.0) /
                    this->palmColWidth[palmIdx], 0.0, 1.0);

                ai = this->palmVerSize[palmIdx] -
                    std::ceil(vPosInCol * this->palmVerSize[palmIdx]) - 1;
                aj = std::ceil(hPosInCol * this->palmHorSize[palmIdx]) - 1;
                ai = std::max(ai, 0);
                aj = std::max(aj, 0);
                aIndex = 20;
                int baseIndex = 13;
                if (ai == 0)
                {
                  // four sensors on first row, and five on the second
                  // so adjust aj for sensors 16 and 17
                  aj = (aj > 2) ? aj - 1 : aj;
                  aIndex = baseIndex + aj;
                }
                else
                  aIndex = baseIndex + ai * (this->palmHorSize[4]-1) + aj;

                _tactileMsg->palm[aIndex] = tactileOuput;
                break;
              }
            }
          }
          // if finger: make sure finger index is valid and
          // contact is on the inside of the hand (palm side)
          else if (fingerIdx != -1 && pos.y > 0)
          {
            // compute finger tactile array index
            vPosInCol = math::clamp((pos.z +
                this->fingerColLength[fingerColIdx]/2)
                / fingerColLength[fingerColIdx], 0.0, 1.0);
            hPosInCol = math::clamp((-pos.x +
                this->fingerColWidth[fingerColIdx]/2)
                / fingerColWidth[fingerColIdx], 0.0, 1.0);

            ai = this->fingerVerSize[fingerColIdx] -
                std::ceil(vPosInCol * this->fingerVerSize[fingerColIdx]) - 1;
            aj = std::ceil(hPosInCol * this->fingerHorSize[fingerColIdx]) - 1;
            ai = std::max(ai, 0);
            aj = std::max(aj, 0);

            aIndex = fingerColIdx * this->fingerHorSize[0]
                * this->fingerVerSize[0] +
                ai * this->fingerHorSize[fingerColIdx] + aj;

            // Set the corresponding index in tactile sensor array to 1
            switch (fingerIdx)
            {
              case 0:
                 _tactileMsg->f0[aIndex] = tactileOuput;
                break;
              case 1:
                 _tactileMsg->f1[aIndex] = tactileOuput;
                break;
              case 2:
                 _tactileMsg->f2[aIndex] = tactileOuput;
                break;
              case 3:
                 _tactileMsg->f3[aIndex] = tactileOuput;
                break;
            }
          }
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::RightTactileConnect()
{
  boost::mutex::scoped_lock lock(this->rightTactileConnectionMutex);
  this->rightTactileConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::LeftTactileConnect()
{
  boost::mutex::scoped_lock lock(this->leftTactileConnectionMutex);
  this->leftTactileConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::RightTactileDisconnect()
{
  boost::mutex::scoped_lock lock(this->rightTactileConnectionMutex);
  this->rightTactileConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
void SandiaHandPlugin::LeftTactileDisconnect()
{
  boost::mutex::scoped_lock lock(this->leftTactileConnectionMutex);
  this->leftTactileConnectCount--;
}

}
