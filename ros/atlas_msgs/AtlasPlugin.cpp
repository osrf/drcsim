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
#include <algorithm>
#include <stdlib.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "AtlasPlugin.h"

// publish separate /atlas/imu topic, to be deprecated
#include "sensor_msgs/Imu.h"

// publish separate /atlas/force_torque_sensors topic, to be deprecated
#include <atlas_msgs/ForceTorqueSensors.h>

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

////////////////////////////////////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
  // the parent link of the imu_sensor ends up being pelvis after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_senosr block.
  this->imuLinkName = "imu_link";

  // initialize behavior library
  this->atlasSimInterface = create_atlas_sim_interface();

  // setup behavior to string map
  this->behaviorMap["User"] = atlas_msgs::AtlasSimInterfaceCommand::USER;
  this->behaviorMap["Stand"] = atlas_msgs::AtlasSimInterfaceCommand::STAND;
  this->behaviorMap["Walk"] = atlas_msgs::AtlasSimInterfaceCommand::WALK;
  this->behaviorMap["Step"] = atlas_msgs::AtlasSimInterfaceCommand::STEP;
  this->behaviorMap["Manipulate"] =
    atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE;
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
  // By default, cheats are off.  Allow override via environment variable.
  char* cheatsEnabledString = getenv("VRC_CHEATS_ENABLED");
  if (cheatsEnabledString && (std::string(cheatsEnabledString) == "1"))
    this->cheatsEnabled = true;
  else
    this->cheatsEnabled = false;

  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();

  // JointController: built-in gazebo to control joints
  this->jointController = this->model->GetJointController();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->jointCmdPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + this->model->GetName() + "/joint_cmd");

  // save sdf
  this->sdf = _sdf;

  // initialize update time
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // init joints, hardcoded for Atlas
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

  // get pointers to joints from gazebo
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

  // get effort limits from gazebo
  this->effortLimit.resize(this->jointNames.size());
  for (unsigned i = 0; i < this->effortLimit.size(); ++i)
    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);

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

  {
    // initialize PID states: error terms
    this->errorTerms.resize(this->joints.size());
    for (unsigned i = 0; i < this->errorTerms.size(); ++i)
    {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }
  }

  {
    // We are not sending names due to the fact that there is an enum
    // joint indices in AtlasState.msg.
    this->atlasState.position.resize(this->joints.size());
    this->atlasState.velocity.resize(this->joints.size());
    this->atlasState.effort.resize(this->joints.size());
    this->atlasState.kp_position.resize(this->joints.size());
    this->atlasState.ki_position.resize(this->joints.size());
    this->atlasState.kd_position.resize(this->joints.size());
    this->atlasState.kp_velocity.resize(this->joints.size());
    this->atlasState.i_effort_min.resize(this->joints.size());
    this->atlasState.i_effort_max.resize(this->joints.size());
    this->atlasState.k_effort.resize(this->joints.size());

    this->jointStates.name.resize(this->joints.size());
    this->jointStates.position.resize(this->joints.size());
    this->jointStates.velocity.resize(this->joints.size());
    this->jointStates.effort.resize(this->joints.size());

    for (unsigned int i = 0; i < this->jointNames.size(); ++i)
      this->jointStates.name[i] = this->jointNames[i];
  }

  {
    this->atlasCommand.position.resize(this->joints.size());
    this->atlasCommand.velocity.resize(this->joints.size());
    this->atlasCommand.effort.resize(this->joints.size());
    this->atlasCommand.kp_position.resize(this->joints.size());
    this->atlasCommand.ki_position.resize(this->joints.size());
    this->atlasCommand.kd_position.resize(this->joints.size());
    this->atlasCommand.kp_velocity.resize(this->joints.size());
    this->atlasCommand.i_effort_min.resize(this->joints.size());
    this->atlasCommand.i_effort_max.resize(this->joints.size());
    this->atlasCommand.k_effort.resize(this->joints.size());

    this->ZeroAtlasCommand();
  }

  {
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
  }

  {
    // AtlasSimInterface:  initialize atlasControlOutput
    for(unsigned int i = 0; i < this->joints.size(); ++i)
      this->atlasControlOutput.f_out[i] = 0;
    this->atlasControlOutput.pos_est.position = AtlasVec3f(0, 0, 0);
    this->atlasControlOutput.pos_est.velocity = AtlasVec3f(0, 0, 0);
    this->atlasControlOutput.foot_pos_est[0] = AtlasVec3f(0, 0, 0);
    this->atlasControlOutput.foot_pos_est[1] = AtlasVec3f(0, 0, 0);
  }

  {
    AtlasBehaviorFeedback *fb = &(this->atlasControlOutput.behavior_feedback);
    fb->status_flags = 0;
    fb->trans_from_behavior_index = 0;
    fb->trans_to_behavior_index = 0;
    fb->stand_feedback.status_flags = 0;
    fb->step_feedback.status_flags = 0;
    fb->walk_feedback.t_step_rem = 0.0;
    fb->walk_feedback.current_step_index = 0;
    fb->walk_feedback.next_step_index_needed = 0;
    fb->walk_feedback.status_flags = 0;
    for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
    {
      fb->walk_feedback.step_data_saturated[i].step_index = 0;
      fb->walk_feedback.step_data_saturated[i].foot_index = 0;
      fb->walk_feedback.step_data_saturated[i].duration = 0.0;
      fb->walk_feedback.step_data_saturated[i].position.n[0] = 0.0;
      fb->walk_feedback.step_data_saturated[i].position.n[1] = 0.0;
      fb->walk_feedback.step_data_saturated[i].position.n[2] = 0.0;
      fb->walk_feedback.step_data_saturated[i].yaw = 0.0;
      fb->walk_feedback.step_data_saturated[i].normal.n[0] = 0.0;
      fb->walk_feedback.step_data_saturated[i].normal.n[1] = 0.0;
      fb->walk_feedback.step_data_saturated[i].normal.n[2] = 0.0;
      fb->walk_feedback.step_data_saturated[i].swing_height = 0.0;
    }
    fb->manipulate_feedback.status_flags = 0;
    fb->manipulate_feedback.clamped.pelvis_height = 0.0;
    fb->manipulate_feedback.clamped.pelvis_yaw = 0.0;
    fb->manipulate_feedback.clamped.pelvis_lat = 0.0;
  }

  {
    // AtlasSimInterface:  initialize atlasRobotState joints data
    this->atlasRobotState.t = this->world->GetSimTime().Double();
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->atlasRobotState.j[i].q = 0;
      this->atlasRobotState.j[i].qd = 0;
      this->atlasRobotState.j[i].f = 0;
    }
    // AtlasSimInterface:  initialize atlasRobotState sensor data
    this->atlasRobotState.imu.imu_timestamp =
      1.0e6 * this->world->GetSimTime().sec +
      1.0e-3 * this->world->GetSimTime().nsec;
    this->atlasRobotState.imu.angular_velocity.n[0] = 0;
    this->atlasRobotState.imu.angular_velocity.n[1] = 0;
    this->atlasRobotState.imu.angular_velocity.n[2] = 0;
    this->atlasRobotState.imu.linear_acceleration.n[0] = 0;
    this->atlasRobotState.imu.linear_acceleration.n[1] = 0;
    this->atlasRobotState.imu.linear_acceleration.n[2] = 0;
    this->atlasRobotState.imu.orientation_estimate.m_qw = 0;
    this->atlasRobotState.imu.orientation_estimate.m_qx = 0;
    this->atlasRobotState.imu.orientation_estimate.m_qy = 0;
    this->atlasRobotState.imu.orientation_estimate.m_qz = 0;
    this->atlasRobotState.foot_sensors[0].fz = 0;
    this->atlasRobotState.foot_sensors[0].mx = 0;
    this->atlasRobotState.foot_sensors[0].my = 0;
    this->atlasRobotState.foot_sensors[1].fz = 0;
    this->atlasRobotState.foot_sensors[1].mx = 0;
    this->atlasRobotState.foot_sensors[1].my = 0;
    this->atlasRobotState.wrist_sensors[0].f.n[0] = 0;
    this->atlasRobotState.wrist_sensors[0].f.n[1] = 0;
    this->atlasRobotState.wrist_sensors[0].f.n[2] = 0;
    this->atlasRobotState.wrist_sensors[0].m.n[0] = 0;
    this->atlasRobotState.wrist_sensors[0].m.n[1] = 0;
    this->atlasRobotState.wrist_sensors[0].m.n[2] = 0;
    this->atlasRobotState.wrist_sensors[1].f.n[0] = 0;
    this->atlasRobotState.wrist_sensors[1].f.n[1] = 0;
    this->atlasRobotState.wrist_sensors[1].f.n[2] = 0;
    this->atlasRobotState.wrist_sensors[1].m.n[0] = 0;
    this->atlasRobotState.wrist_sensors[1].m.n[1] = 0;
    this->atlasRobotState.wrist_sensors[1].m.n[2] = 0;
  }

  {
    // internal pid params
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->atlasControlInput.j[i].q_d = 0.0;
      this->atlasControlInput.j[i].qd_d = 0.0;
      this->atlasControlInput.j[i].f_d = 0.0;
      this->atlasControlInput.jparams[i].k_q_p = 0.0;
      this->atlasControlInput.jparams[i].k_q_i = 0.0;
      this->atlasControlInput.jparams[i].k_qd_p = 0.0;
    }
    // stand
    this->atlasControlInput.stand_params.placeholder = 0;
    // step
    AtlasBehaviorStepParams *stepParams =
      &this->atlasControlInput.step_params;
    stepParams->desired_step.step_index = 1;
    stepParams->desired_step.foot_index = 0;
    stepParams->desired_step.duration = 0;
    stepParams->desired_step.position = AtlasVec3f(0, 0, 0);
    stepParams->desired_step.yaw = 0.0;
    stepParams->desired_step.normal = AtlasVec3f(0, 0, 0);
    stepParams->desired_step.swing_height = 0.0;
    stepParams->use_demo_walk = false;
    // walk
    AtlasBehaviorWalkParams *walkParams =
      &this->atlasControlInput.walk_params;
    for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS; ++stepId)
    {
      walkParams->step_data[stepId].step_index = stepId + 1;
      walkParams->step_data[stepId].foot_index = 0;
      walkParams->step_data[stepId].duration = 0.0;
      walkParams->step_data[stepId].position = AtlasVec3f(0, 0, 0);
      walkParams->step_data[stepId].yaw = 0;
      walkParams->step_data[stepId].normal = AtlasVec3f(0, 0, 0);
      walkParams->step_data[stepId].swing_height = 0.0;
      walkParams->use_demo_walk = false;
    }
    walkParams->use_demo_walk = false;
    // manipulate
    AtlasBehaviorManipulateParams *manipulateParams =
      &this->atlasControlInput.manipulate_params;
    manipulateParams->use_desired = false;
    manipulateParams->desired.pelvis_height = 0.0;
    manipulateParams->desired.pelvis_yaw = 0.0;
    manipulateParams->desired.pelvis_lat = 0.0;
    manipulateParams->use_demo_mode = false;
  }

  {
    // initialize AtlasSimInterfaceState
    this->asiState.error_code = atlas_msgs::AtlasSimInterfaceState::NO_ERRORS;
    this->asiState.current_behavior = -1;
    this->asiState.desired_behavior = -1;
    for(unsigned int i = 0; i < this->jointNames.size(); ++i)
      this->asiState.f_out[i] = 0.0;
    this->asiState.pos_est.position.x = 0.0;
    this->asiState.pos_est.position.y = 0.0;
    this->asiState.pos_est.position.z = 0.0;
    this->asiState.pos_est.velocity.x = 0.0;
    this->asiState.pos_est.velocity.y = 0.0;
    this->asiState.pos_est.velocity.z = 0.0;
    for (unsigned int i = 0; i < Atlas::NUM_FEET; ++i)
    {
      this->asiState.foot_pos_est[i].position.x = 0.0;
      this->asiState.foot_pos_est[i].position.y = 0.0;
      this->asiState.foot_pos_est[i].position.z = 0.0;
      this->asiState.foot_pos_est[i].orientation.w = 1.0;
      this->asiState.foot_pos_est[i].orientation.x = 0.0;
      this->asiState.foot_pos_est[i].orientation.y = 0.0;
      this->asiState.foot_pos_est[i].orientation.z = 0.0;
    }
    {
      atlas_msgs::AtlasBehaviorFeedback *fb =
        &(this->asiState.behavior_feedback);
      fb->status_flags = atlas_msgs::AtlasBehaviorFeedback::STATUS_OK;
      fb->trans_from_behavior_index = 0;
      fb->trans_to_behavior_index = 0;
      fb->stand_feedback.status_flags = 0;
      fb->step_feedback.status_flags = 0;
      fb->walk_feedback.t_step_rem = 0.0;
      fb->walk_feedback.current_step_index = 0;
      fb->walk_feedback.next_step_index_needed = 0;
      fb->walk_feedback.status_flags = 0;
      for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
      {
        fb->walk_feedback.step_data_saturated[i].step_index = 0;
        fb->walk_feedback.step_data_saturated[i].foot_index = 0;
        fb->walk_feedback.step_data_saturated[i].duration = 0.0;
        fb->walk_feedback.step_data_saturated[i].pose = geometry_msgs::Pose();
        fb->walk_feedback.step_data_saturated[i].swing_height = 0.0;
      }
      fb->manipulate_feedback.status_flags = 0;
      fb->manipulate_feedback.clamped.pelvis_height = 0.0;
      fb->manipulate_feedback.clamped.pelvis_yaw = 0.0;
      fb->manipulate_feedback.clamped.pelvis_lat = 0.0;
    }

    // start with PID control
    this->asiState.k_effort.resize(this->jointNames.size());
    for(unsigned int i = 0; i < this->jointNames.size(); ++i)
      this->asiState.k_effort[i] = 255;
  }

  // AtlasSimInterface:
  // Calling into the behavior library to reset controls and set startup
  // behavior.
  this->asiState.error_code = this->atlasSimInterface->reset_control();
  if (this->asiState.error_code != NO_ERRORS)
    ROS_ERROR("AtlasSimInterface: reset controls on startup failed with "
              "error code (%d).", this->asiState.error_code);
  this->asiState.error_code =
    this->atlasSimInterface->set_desired_behavior("User");
  if (this->asiState.error_code != NO_ERRORS)
    ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
              "error code (%d).", this->asiState.error_code);

  this->asiState.desired_behavior = atlas_msgs::AtlasSimInterfaceCommand::USER;

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

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&AtlasPlugin::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::Pause(
  const std_msgs::String::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->pauseMutex);
  this->pause.notify_one();
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetAtlasCommand(
  const atlas_msgs::AtlasCommand::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->atlasCommand.header.stamp = _msg->header.stamp;

  // for atlasCommand, only position, velocity and efforts are used.
  if (_msg->position.size() == this->atlasCommand.position.size())
    std::copy(_msg->position.begin(), _msg->position.end(),
      this->atlasCommand.position.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg->position.size(), this->atlasCommand.position.size());

  if (_msg->velocity.size() == this->atlasCommand.velocity.size())
    std::copy(_msg->velocity.begin(), _msg->velocity.end(),
      this->atlasCommand.velocity.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg->velocity.size(), this->atlasCommand.velocity.size());

  if (_msg->effort.size() == this->atlasCommand.effort.size())
    std::copy(_msg->effort.begin(), _msg->effort.end(),
      this->atlasCommand.effort.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->atlasCommand.effort.size());

  // the rest are stored in atlasState for publication
  if (_msg->kp_position.size() == this->atlasState.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->atlasState.kp_position.size());

  if (_msg->ki_position.size() == this->atlasState.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->atlasState.ki_position.size());

  if (_msg->kd_position.size() == this->atlasState.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->atlasState.kd_position.size());

  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->atlasState.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->atlasState.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->atlasState.i_effort_max.size());

  if (_msg->k_effort.size() == this->atlasState.k_effort.size())
    std::copy(_msg->k_effort.begin(), _msg->k_effort.end(),
      this->atlasState.k_effort.begin());
  else
    ROS_DEBUG("AtlasCommand message contains different number of"
      " elements k_effort[%ld] than expected[%ld]",
      _msg->k_effort.size(), this->atlasState.k_effort.size());
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->atlasCommand.header.stamp = _msg->header.stamp;

  /// \TODO: at some point, we can try stuffing
  ///   AtlasControlInput::J and AtlasControlInput::jparams
  /// to test out BDI internal PID controller
  /// as a replacement to PID control in AtlasPlugin

  if (_msg->position.size() == this->atlasCommand.position.size())
    std::copy(_msg->position.begin(), _msg->position.end(),
      this->atlasCommand.position.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg->position.size(), this->atlasCommand.position.size());

  if (_msg->velocity.size() == this->atlasCommand.velocity.size())
    std::copy(_msg->velocity.begin(), _msg->velocity.end(),
      this->atlasCommand.velocity.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg->velocity.size(), this->atlasCommand.velocity.size());

  if (_msg->effort.size() == this->atlasCommand.effort.size())
    std::copy(_msg->effort.begin(), _msg->effort.end(),
      this->atlasCommand.effort.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->atlasCommand.effort.size());

  if (_msg->kp_position.size() == this->atlasState.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->atlasState.kp_position.size());

  if (_msg->ki_position.size() == this->atlasState.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->atlasState.ki_position.size());

  if (_msg->kd_position.size() == this->atlasState.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->atlasState.kd_position.size());

  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->atlasState.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->atlasState.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  else
    ROS_DEBUG("JointCommands message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->atlasState.i_effort_max.size());
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

  // publish multi queue
  this->pmq.startServiceThread();

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // Get window size from ros parameter server (seconds)
  if (!this->rosNode->getParam(
    "atlas_controller/statistics_time_window_size",
    this->atlasCommandAgeBufferDuration))
  {
    this->atlasCommandAgeBufferDuration = 1.0;
    ROS_INFO("controller statistics window size not specified in"
             " ros parameter server, defaulting to %f sec.",
             this->atlasCommandAgeBufferDuration);
  }
  double stepSize = this->world->GetPhysicsEngine()->GetMaxStepSize();
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
  unsigned int bufferSize = this->atlasCommandAgeBufferDuration / stepSize;
  this->atlasCommandAgeBuffer.resize(bufferSize);
  this->atlasCommandAgeDelta2Buffer.resize(bufferSize);
  this->atlasCommandAgeBufferIndex = 0;
  this->atlasCommandAgeMean = 0.0;
  this->atlasCommandAgeVariance = 0.0;

  // ROS Controller API
  /// brief broadcasts the robot states
  this->pubJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "atlas/joint_states", 1);

  this->pubAtlasStateQueue = this->pmq.addPub<atlas_msgs::AtlasState>();
  this->pubAtlasState = this->rosNode->advertise<atlas_msgs::AtlasState>(
    "atlas/atlas_state", 1);

  // publish separate /atlas/imu topic, to be deprecated
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>("atlas/imu", 10);
  this->pubImuQueue = this->pmq.addPub<sensor_msgs::Imu>();

  // publish separate /atlas/force_torque_sensors topic, to be deprecated
  this->pubForceTorqueSensorsQueue =
    this->pmq.addPub<atlas_msgs::ForceTorqueSensors>();
  this->pubForceTorqueSensors =
    this->rosNode->advertise<atlas_msgs::ForceTorqueSensors>(
    "atlas/force_torque_sensors", 10);

  // ros publication / subscription
  this->pubControllerStatisticsQueue =
    this->pmq.addPub<atlas_msgs::ControllerStatistics>();
  this->pubControllerStatistics =
    this->rosNode->advertise<atlas_msgs::ControllerStatistics>(
    "atlas/controller_statistics", 10);

  if (this->cheatsEnabled)
  {
    // these topics are used for debugging only
    this->pubLFootContact =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "atlas/debug/l_foot_contact", 10);
    this->pubLFootContactQueue = 
      this->pmq.addPub<geometry_msgs::WrenchStamped>();
  
    // these topics are used for debugging only
    this->pubRFootContact =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "atlas/debug/r_foot_contact", 10);
    this->pubRFootContactQueue = 
      this->pmq.addPub<geometry_msgs::WrenchStamped>();

    // ros topic subscribtions
    ros::SubscribeOptions pauseSo =
      ros::SubscribeOptions::create<std_msgs::String>(
      "atlas/pause", 1,
      boost::bind(&AtlasPlugin::Pause, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    pauseSo.transport_hints =
      ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
    this->subPause =
      this->rosNode->subscribe(pauseSo);
  }

  // ros topic subscribtions
  ros::SubscribeOptions atlasCommandSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasCommand>(
    "atlas/atlas_command", 1,
    boost::bind(&AtlasPlugin::SetAtlasCommand, this, _1),
    ros::VoidPtr(), &this->rosQueue);

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint commands, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  atlasCommandSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  this->subAtlasCommand =
    this->rosNode->subscribe(atlasCommandSo);

  // ros topic subscribtions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "atlas/joint_commands", 1,
    boost::bind(&AtlasPlugin::SetJointCommands, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  // This subscription is TCP because the message is larger than a UDP datagram
  // and we have had reports of corrupted data, which we attribute to erroneous
  // demarshalling following packet loss.
  jointCommandsSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  this->subJointCommands =
    this->rosNode->subscribe(jointCommandsSo);

  if (this->cheatsEnabled)
  {
    // ros topic subscribtions
    ros::SubscribeOptions testSo =
      ros::SubscribeOptions::create<atlas_msgs::Test>(
      "atlas/debug/test", 1,
      boost::bind(&AtlasPlugin::SetExperimentalDampingPID, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subTest = this->rosNode->subscribe(testSo);
  }

  // initialize status pub time
  this->lastControllerStatisticsTime = this->world->GetSimTime().Double();

  // controller statistics update rate defaults to 1kHz,
  // read from ros param if available
  double rate;
  if (this->rosNode->getParam("atlas/controller_statistics/update_rate",
    rate))
  {
    rate = math::clamp(rate, 1.0, 10000.0);
    ROS_DEBUG("AtlasPlugin controller statistics %f kHz", rate);
    this->statsUpdateRate = rate;
  }
  else
  {
    ROS_DEBUG("AtlasPlugin default controller statistics 1kHz");
    this->statsUpdateRate = 1000.0;
  }

  // AtlasSimInterface:
  // subscribe to a control_mode string message, current valid commands are:
  //   Walk, Stand, Freeze, StandPrep, User
  // the command is passed to the AtlasSimInterface library.
  ros::SubscribeOptions atlasControlModeSo =
    ros::SubscribeOptions::create<std_msgs::String>(
    "atlas/control_mode", 100,
    boost::bind(&AtlasPlugin::OnRobotMode, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subAtlasControlMode = this->rosNode->subscribe(atlasControlModeSo);

  // AtlasSimInterface:
  // closing the loop on BDI Dynamic Behavior Library
  ros::SubscribeOptions asiCommandSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasSimInterfaceCommand>(
    "atlas/atlas_sim_interface_command", 1,
    boost::bind(&AtlasPlugin::SetASICommand, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  asiCommandSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  this->subASICommand = this->rosNode->subscribe(asiCommandSo);

  // AtlasSimInterface:
  // closing the loop on BDI Dynamic Behavior Library
  this->pubASIStateQueue =
    this->pmq.addPub<atlas_msgs::AtlasSimInterfaceState>();
  this->pubASIState =
    this->rosNode->advertise<atlas_msgs::AtlasSimInterfaceState>(
    "atlas/atlas_sim_interface_state", 1);

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&AtlasPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&AtlasPlugin::UpdateStates, this));

  if (this->cheatsEnabled)
  {
    // on contact
    this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
       boost::bind(&AtlasPlugin::OnLContactUpdate, this));
  
    this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
       boost::bind(&AtlasPlugin::OnRContactUpdate, this));
  }

  // Advertise services on the custom queue
  ros::AdvertiseServiceOptions resetControlsAso =
    ros::AdvertiseServiceOptions::create<atlas_msgs::ResetControls>(
      "atlas/reset_controls", boost::bind(
        &AtlasPlugin::ResetControls, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);
  this->resetControlsService = this->rosNode->advertiseService(
    resetControlsAso);
}

////////////////////////////////////////////////////////////////////////////////
bool AtlasPlugin::ResetControls(atlas_msgs::ResetControls::Request &_req,
  atlas_msgs::ResetControls::Response &_res)
{
  boost::mutex::scoped_lock lock(this->mutex);

  _res.success = true;
  _res.status_message = "success";

  if (_req.reset_bdi_controller)
  {
    this->asiState.error_code = this->atlasSimInterface->reset_control();
    if (this->asiState.error_code != NO_ERRORS)
    {
      ROS_ERROR("AtlasSimInterface: reset controls on startup failed with "
                "error code (%d).", this->asiState.error_code);
      _res.success = false;
      _res.status_message = "failed to AtlasSimInterface::reset_control()";
    }
  }

  if (_req.reset_pid_controller)
    for (unsigned i = 0; i < this->errorTerms.size(); ++i)
    {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }

  if (_req.reload_pid_from_ros)
    this->LoadPIDGainsFromParameter();
  else
  {
    // boost::shared_ptr<atlas_msgs::AtlasCommand> msg(_req.atlas_command);
    this->SetAtlasCommand(
      static_cast<atlas_msgs::AtlasCommand::ConstPtr>(&(_req.atlas_command)));
  }

  return _res.success;
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetASICommand(
  const atlas_msgs::AtlasSimInterfaceCommand::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->asiMutex);

  // copy _msg contents directly into
  // atlasControlInput::stand_params
  // atlasControlInput::step_params
  // atlasControlInput::walk_params
  // atlasControlInput::manipulate_params
  // atlasState::k_effort
  // asiState::desired_behavior

  this->asiState.desired_behavior = _msg->behavior;

  // stand
  this->atlasControlInput.stand_params.placeholder = 0;
  // step
  AtlasBehaviorStepParams *stepParams =
    &this->atlasControlInput.step_params;
  stepParams->desired_step.step_index   =
    _msg->step_params.desired_step.step_index;
  stepParams->desired_step.foot_index   =
    _msg->step_params.desired_step.foot_index;
  stepParams->desired_step.duration     =
    _msg->step_params.desired_step.duration;

  stepParams->desired_step.position     =
    this->ToVec3(_msg->step_params.desired_step.pose.position);
  stepParams->desired_step.yaw          = this->ToPose(
    _msg->step_params.desired_step.pose).rot.GetYaw();
  stepParams->desired_step.normal       = this->ToVec3(this->ToPose(
    _msg->step_params.desired_step.pose).rot.RotateVector(
    math::Vector3(0, 0, 1)));

  stepParams->desired_step.swing_height =
    _msg->step_params.desired_step.swing_height;
  stepParams->use_demo_walk =
    _msg->step_params.use_demo_walk;

  // walk
  AtlasBehaviorWalkParams *walkParams =
    &this->atlasControlInput.walk_params;
  for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS; ++stepId)
  {
    walkParams->step_data[stepId].step_index =
      _msg->walk_params.step_data[stepId].step_index;
    walkParams->step_data[stepId].foot_index =
      _msg->walk_params.step_data[stepId].foot_index;
    walkParams->step_data[stepId].duration =
      _msg->walk_params.step_data[stepId].duration;

    walkParams->step_data[stepId].position = this->ToVec3(
      _msg->walk_params.step_data[stepId].pose.position);
    walkParams->step_data[stepId].yaw = this->ToPose(
      _msg->walk_params.step_data[stepId].pose).rot.GetYaw();
    walkParams->step_data[stepId].normal = this->ToVec3(this->ToPose(
      _msg->walk_params.step_data[stepId].pose).rot.RotateVector(
      math::Vector3(0, 0, 1)));

    walkParams->step_data[stepId].swing_height =
      _msg->walk_params.step_data[stepId].swing_height;
  }
  walkParams->use_demo_walk = _msg->walk_params.use_demo_walk;

  // manipulate
  AtlasBehaviorManipulateParams *manipulateParams =
    &this->atlasControlInput.manipulate_params;
  manipulateParams->use_desired =
    _msg->manipulate_params.use_desired;
  manipulateParams->desired.pelvis_height =
    _msg->manipulate_params.desired.pelvis_height;
  manipulateParams->desired.pelvis_yaw =
    _msg->manipulate_params.desired.pelvis_yaw;
  manipulateParams->desired.pelvis_lat =
    _msg->manipulate_params.desired.pelvis_lat;
  manipulateParams->use_demo_mode = false;
    _msg->manipulate_params.use_demo_mode;

  // k_effort
  if (_msg->k_effort.size() == this->atlasState.k_effort.size())
    std::copy(_msg->k_effort.begin(), _msg->k_effort.end(),
      this->atlasState.k_effort.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements k_effort[%ld] than expected[%ld]",
      _msg->k_effort.size(), this->atlasState.k_effort.size());

  /// \TODO: Set atlasControlInput from _msg
  /*
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->atlasControlInput.j[i].q_d = 0.0;
    this->atlasControlInput.j[i].qd_d = 0.0;
    this->atlasControlInput.j[i].f_d = 0.0;
    this->atlasControlInput.jparams[i].k_q_p = 0.0;
    this->atlasControlInput.jparams[i].k_q_i = 0.0;
    this->atlasControlInput.jparams[i].k_qd_p = 0.0;
  }
  */

  // Try and set desired behavior (reverse map of behaviorMap)
  switch (this->asiState.desired_behavior)
  {
    case atlas_msgs::AtlasSimInterfaceCommand::USER:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("User");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::FREEZE:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("Freeze");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("StandPrep");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("Stand");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::WALK:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("Walk");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::STEP:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("Step");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:
      this->asiState.error_code =
        this->atlasSimInterface->set_desired_behavior("Manipulate");
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: setting mode User on startup failed with "
                  "error code (%d).", this->asiState.error_code);
      break;
    default:
      gzerr << "Unrecognized behavior\n";
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  double dt = (curTime - this->lastControllerUpdateTime).Double();

  if (curTime > this->lastControllerUpdateTime)
  {

    // AtlasSimInterface:
    // populate atlasRobotState from robot
    this->atlasRobotState.t = curTime.Double();
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->atlasRobotState.j[i].q = this->joints[i]->GetAngle(0).Radian();
      this->atlasRobotState.j[i].qd = this->joints[i]->GetVelocity(0);
      // wait to fill in this->atlasRobotState.j[i].f later
    }

    // get imu data from imu sensor
    this->GetIMUState(curTime);

    // get force torque sensor data from sensor
    this->GetForceTorqueSensorState(curTime);

    // populate atlasState from robot
    this->atlasState.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->jointStates.header.stamp = this->atlasState.header.stamp;

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->atlasState.position[i] = this->joints[i]->GetAngle(0).Radian();
      this->atlasState.velocity[i] = this->joints[i]->GetVelocity(0);
    }
    // copy from atlasState.position into jointStates.position
    GZ_ASSERT(this->atlasState.position.size() ==
              this->jointStates.position.size(),
              "atlasState.position and "
              "jointStates.position size mismatch.");
    std::copy(this->atlasState.position.begin(),
              this->atlasState.position.end(),
              this->jointStates.position.begin());
    // copy from atlasState.velocity into jointStates.velocity
    GZ_ASSERT(this->atlasState.velocity.size() ==
              this->jointStates.velocity.size(),
              "atlasState.velocity and "
              "jointStates.velocity size mismatch.");
    std::copy(this->atlasState.velocity.begin(),
              this->atlasState.velocity.end(),
              this->jointStates.velocity.begin());

    // AtlasSimInterface:
    // skip the first step, or else, BDI controller might init with bad
    // states and fail:
    if (curTime.Double() > this->world->GetPhysicsEngine()->GetMaxStepSize())
    {
      boost::mutex::scoped_lock lock(this->asiMutex);

      // Try and get desired behavior
	    std::string behaviorStr;
      this->asiState.error_code =
        this->atlasSimInterface->get_desired_behavior(behaviorStr);
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: getting desired behavior returned "
                  "error [%s].",
        this->atlasSimInterface->get_error_code_text(
          (AtlasErrorCode)(this->asiState.error_code)).c_str());
	    if (this->asiState.desired_behavior != this->behaviorMap[behaviorStr])
	    {
	      // debug bdi controller behavior
	      ROS_DEBUG("setting desired behavior[%d] did not change result of "
                  "get_desired_behavior[%d], not implemented?",
                  this->asiState.desired_behavior,
                  this->behaviorMap[behaviorStr]);
	    }

      // Try and get current behavior
      this->asiState.error_code =
        this->atlasSimInterface->get_current_behavior(behaviorStr);
      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: getting current behavior returned "
                  "error [%s].",
        this->atlasSimInterface->get_error_code_text(
          (AtlasErrorCode)(this->asiState.error_code)).c_str());
	    this->asiState.current_behavior = this->behaviorMap[behaviorStr];

      // if current behavior is not desired behavior, controller is in
      // a state of transition.

      this->asiState.error_code =
        this->atlasSimInterface->process_control_input(
        this->atlasControlInput, this->atlasRobotState,
        this->atlasControlOutput);

      if (this->asiState.error_code != NO_ERRORS)
        ROS_ERROR("AtlasSimInterface: process_control_input returned "
                  "error [%s].",
        this->atlasSimInterface->get_error_code_text(
          (AtlasErrorCode)(this->asiState.error_code)).c_str());

      // fill in rest of asiState
      std::copy(this->atlasControlOutput.f_out,
                this->atlasControlOutput.f_out+this->jointNames.size(),
                this->asiState.f_out.begin());
      {
        // initialize AtlasSimInterfaceState
        this->asiState.pos_est.position =
          this->ToGeomVec3(this->atlasControlOutput.pos_est.position);
        this->asiState.pos_est.velocity =
          this->ToGeomVec3(this->atlasControlOutput.pos_est.velocity);
        for (unsigned int i = 0; i < Atlas::NUM_FEET; ++i)
        {
          this->asiState.foot_pos_est[i].position =
            this->ToPoint(this->atlasControlOutput.foot_pos_est[i]);
          this->asiState.foot_pos_est[i].orientation =
            this->ToQ(math::Quaternion(
            this->atlasRobotState.imu.orientation_estimate.m_qw,
            this->atlasRobotState.imu.orientation_estimate.m_qx,
            this->atlasRobotState.imu.orientation_estimate.m_qy,
            this->atlasRobotState.imu.orientation_estimate.m_qz));
        }

        this->AtlasControlOutputToAtlasSimInterfaceState(
          &(this->asiState.behavior_feedback),
          &(this->atlasControlOutput.behavior_feedback));

        // start with PID control
        this->asiState.k_effort.resize(this->jointNames.size());
        for(unsigned int i = 0; i < this->jointNames.size(); ++i)
          this->asiState.k_effort[i] = this->atlasState.k_effort[i];
      }

      // 80 characters
      atlas_msgs::AtlasBehaviorFeedback *fb =
        &(this->asiState.behavior_feedback);
      AtlasBehaviorFeedback *fbOut =
        &(this->atlasControlOutput.behavior_feedback);

      // just copying
      fb->status_flags = fbOut->status_flags;
      fb->trans_from_behavior_index = fbOut->trans_from_behavior_index;
      fb->trans_to_behavior_index = fbOut->trans_to_behavior_index;

      // do something based on current_behavior
      switch (this->asiState.current_behavior)
      {
        case atlas_msgs::AtlasSimInterfaceCommand::USER:
          {
          }
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::STAND:
          {
            fb->stand_feedback.status_flags =
              fbOut->stand_feedback.status_flags;
          }
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::FREEZE:
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP:
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::WALK:
          {
            fb->walk_feedback.t_step_rem = fbOut->walk_feedback.t_step_rem;
            fb->walk_feedback.current_step_index =
              fbOut->walk_feedback.current_step_index;
            fb->walk_feedback.next_step_index_needed =
              fbOut->walk_feedback.next_step_index_needed;
            fb->walk_feedback.status_flags = fbOut->walk_feedback.status_flags;
            for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
            {
              atlas_msgs::AtlasBehaviorStepData *sd =
                &(fb->walk_feedback.step_data_saturated[i]);
              AtlasBehaviorStepData *sdOut =
                &(fbOut->walk_feedback.step_data_saturated[i]);
              sd->step_index = sdOut->step_index;
              sd->foot_index = sdOut->foot_index;
              sd->duration = sdOut->duration;

              // compose geometry_msgs::Pose from position, yaw, normal
              sd->pose.position = this->ToPoint(sdOut->position);
              sd->pose.orientation = this->ToQ(
                math::Quaternion(0, 0, sdOut->yaw));
              // \TODO: further rotate rot based on normal
              // sd->pose.rot = sdOut->normal ...;

              sdOut->swing_height = sd->swing_height;
            }
            // gzdbg << " csi[" << fb->walk_feedback.current_step_index
            //       << "] nsi[" << fb->walk_feedback.next_step_index_needed
            //       << "] flag[" << fb->walk_feedback.status_flags
            //       << "]\n";

            // demo debug
            if (this->atlasControlInput.walk_params.use_demo_walk)
            {
              static const unsigned int lastStep = 25;
              // or if status_flag turns from 2 to 4
              if (fb->walk_feedback.current_step_index == lastStep)
              {
                this->atlasSimInterface->set_desired_behavior("Stand");
                this->asiState.desired_behavior =
                  atlas_msgs::AtlasSimInterfaceCommand::STAND;
              }
            }
          }
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::STEP:
          {
            fb->step_feedback.status_flags =
              fbOut->step_feedback.status_flags;
          }
          break;
        case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:
          {
            fb->stand_feedback.status_flags =
              fbOut->stand_feedback.status_flags;
          }
          break;
        default:
          break;
      }
      // set asiState and publish asiState
      this->pubASIStateQueue->push(this->asiState, this->pubASIState);
    }

    {
      boost::mutex::scoped_lock lock(this->mutex);
      {
        // Keep track of age of atlasCommand age in seconds.
        // Note the value is invalid as a moving window average age
        // until the buffer is full.
        this->atlasCommandAge = curTime.Double() -
          this->atlasCommand.header.stamp.toSec();

        double weightedAtlasCommandAge = this->atlasCommandAge
          / this->atlasCommandAgeBuffer.size();

        // for variance calculation, save delta before average is updated.
        double delta = this->atlasCommandAge - this->atlasCommandAgeMean;

        // update average
        this->atlasCommandAgeMean += weightedAtlasCommandAge;
        this->atlasCommandAgeMean -=
          this->atlasCommandAgeBuffer[this->atlasCommandAgeBufferIndex];

        // update variance with new average
        double delta2 = delta *
          (this->atlasCommandAge - this->atlasCommandAgeMean);
        this->atlasCommandAgeVariance += delta2;
        this->atlasCommandAgeVariance -=
          this->atlasCommandAgeDelta2Buffer[
          this->atlasCommandAgeBufferIndex];

        // save weighted average in window
        this->atlasCommandAgeBuffer[this->atlasCommandAgeBufferIndex] =
          weightedAtlasCommandAge;

        // save delta buffer for incremental variance calculation
        this->atlasCommandAgeDelta2Buffer[
          this->atlasCommandAgeBufferIndex] = delta2;

        this->atlasCommandAgeBufferIndex =
         (this->atlasCommandAgeBufferIndex + 1) %
         this->atlasCommandAgeBuffer.size();
      }

      /// update pid with feedforward force
      for (unsigned int i = 0; i < this->joints.size(); ++i)
      {
        // truncate joint position within range of motion
        double positionTarget = math::clamp(
          this->atlasCommand.position[i],
          this->joints[i]->GetLowStop(0).Radian(),
          this->joints[i]->GetHighStop(0).Radian());

        double q_p = positionTarget - this->atlasState.position[i];

        if (!math::equal(dt, 0.0))
          this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

        this->errorTerms[i].q_p = q_p;

        this->errorTerms[i].qd_p =
          this->atlasCommand.velocity[i] - this->atlasState.velocity[i];

        this->errorTerms[i].k_i_q_i = math::clamp(
          this->errorTerms[i].k_i_q_i +
          dt * this->atlasState.ki_position[i] * this->errorTerms[i].q_p,
          static_cast<double>(this->atlasState.i_effort_min[i]),
          static_cast<double>(this->atlasState.i_effort_max[i]));

        // convert k_effort to a double between 0 and 1
        double k_effort =
          static_cast<double>(this->atlasState.k_effort[i])/255.0;

        // use gain params to compute force cmd
        // AtlasSimInterface:  also, add bdi controller feed forward force
        // to overall control torque scaled by 1 - k_effort.
        double forceUnclamped =
          k_effort * (
          this->atlasState.kp_position[i] * this->errorTerms[i].q_p +
                                            this->errorTerms[i].k_i_q_i +
          this->atlasState.kd_position[i] * this->errorTerms[i].d_q_p_dt +
          this->atlasState.kp_velocity[i] * this->errorTerms[i].qd_p +
                                            this->atlasCommand.effort[i]) +
          (1.0 - k_effort)                * this->atlasControlOutput.f_out[i];

        // keep unclamped force for integral tie-back calculation
        double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i],
          this->effortLimit[i]);

        // integral tie-back during control saturation if using integral gain
        if (!math::equal(forceClamped,forceUnclamped) &&
            !math::equal((double)this->atlasState.ki_position[i],0.0) )
        {
          // lock integral term to provide continuous control as system moves
          // out of staturation
          this->errorTerms[i].k_i_q_i = math::clamp(
            this->errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
          static_cast<double>(this->atlasState.i_effort_min[i]),
          static_cast<double>(this->atlasState.i_effort_max[i]));
        }

        // clamp force after integral tie-back
        forceClamped = math::clamp(forceUnclamped,
          -this->effortLimit[i], this->effortLimit[i]);

        // apply force to joint
        this->joints[i]->SetForce(0, forceClamped);

        // fill in jointState efforts
        this->atlasState.effort[i] = forceClamped;
        this->jointStates.effort[i] = forceClamped;

        // AtlasSimInterface: fill in atlasRobotState efforts.
        // FIXME: Is this used by the controller?  i.e. should this happen
        // before process_control_input?
        this->atlasRobotState.j[i].f = forceClamped;
      }
    }
    this->lastControllerUpdateTime = curTime;

    this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);
    this->pubAtlasStateQueue->push(this->atlasState, this->pubAtlasState);

    /// controller statistics diagnostics, damages, etc.
    if (this->pubControllerStatistics.getNumSubscribers() > 0)
    {
      if ((curTime - this->lastControllerStatisticsTime).Double() >=
        1.0/this->statsUpdateRate)
      {
        atlas_msgs::ControllerStatistics msg;
        msg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
        msg.command_age = this->atlasCommandAge;
        msg.command_age_mean = this->atlasCommandAgeMean;
        msg.command_age_variance = this->atlasCommandAgeVariance /
          (this->atlasCommandAgeBuffer.size() - 1);
        msg.command_age_window_size = this->atlasCommandAgeBufferDuration;

        this->pubControllerStatisticsQueue->push(msg,
          this->pubControllerStatistics);
        this->lastControllerStatisticsTime = curTime;
      }
    }
  }

  // EXPERIMENTAL: wait for controller publication?
  // {
  //   boost::mutex::scoped_lock lock(this->pauseMutex);
  //   // pause.wait(lock);
  //   boost::system_time timeout = boost::get_system_time() +
  //     boost::posix_time::milliseconds(0.5);
  //   pause.timed_wait(lock, timeout);
  // }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::OnLContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->lFootContactSensor->GetContacts();

  geometry_msgs::WrenchStamped msg;
  math::Vector3 fTotal;
  math::Vector3 tTotal;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    msg.header.stamp = ros::Time(contacts.contact(i).time().sec(),
                                 contacts.contact(i).time().nsec());
    msg.header.frame_id = "l_foot";
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
    fTotal.Set(0, 0, 0);
    tTotal.Set(0, 0, 0);
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
    msg.wrench.force.x = fTotal.x;
    msg.wrench.force.y = fTotal.y;
    msg.wrench.force.z = fTotal.z;
    msg.wrench.torque.x = tTotal.x;
    msg.wrench.torque.y = tTotal.y;
    msg.wrench.torque.z = tTotal.z;
    this->pubLFootContactQueue->push(msg, this->pubLFootContact);
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::OnRContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->rFootContactSensor->GetContacts();

  geometry_msgs::WrenchStamped msg;
  math::Vector3 fTotal;
  math::Vector3 tTotal;

  // GetContacts returns all contacts on the collision body
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // loop through all contact pairs to sum the total force
    // on collision1

    msg.header.stamp = ros::Time(contacts.contact(i).time().sec(),
                                 contacts.contact(i).time().nsec());
    msg.header.frame_id = "r_foot";
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
    fTotal.Set(0, 0, 0);
    tTotal.Set(0, 0, 0);
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      // loop through all contacts between collision1 and collision2

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
    msg.wrench.force.x = fTotal.x;
    msg.wrench.force.y = fTotal.y;
    msg.wrench.force.z = fTotal.z;
    msg.wrench.torque.x = tTotal.x;
    msg.wrench.torque.y = tTotal.y;
    msg.wrench.torque.z = tTotal.z;
    this->pubRFootContactQueue->push(msg, this->pubRFootContact);
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::ZeroAtlasCommand()
{
  for (unsigned i = 0; i < this->jointNames.size(); ++i)
  {
    this->atlasCommand.position[i] = 0;
    this->atlasCommand.velocity[i] = 0;
    this->atlasCommand.effort[i] = 0;
    // store these directly on altasState, more efficient for pub later
    this->atlasState.kp_position[i] = 0;
    this->atlasState.ki_position[i] = 0;
    this->atlasState.kd_position[i] = 0;
    this->atlasState.kp_velocity[i] = 0;
    this->atlasState.i_effort_min[i] = 0;
    this->atlasState.i_effort_max[i] = 0;
    this->atlasState.k_effort[i] = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::ZeroJointCommands()
{
  for (unsigned i = 0; i < this->jointNames.size(); ++i)
  {
    this->jointCommands.position[i] = 0;
    this->jointCommands.velocity[i] = 0;
    this->jointCommands.effort[i] = 0;
    // store these directly on altasState, more efficient for pub later
    this->atlasState.kp_position[i] = 0;
    this->atlasState.ki_position[i] = 0;
    this->atlasState.kd_position[i] = 0;
    this->atlasState.kp_velocity[i] = 0;
    this->atlasState.i_effort_min[i] = 0;
    this->atlasState.i_effort_max[i] = 0;
    this->atlasState.k_effort[i] = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::LoadPIDGainsFromParameter()
{
  // pull down controller parameters
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    char joint_ns[200] = "";
    snprintf(joint_ns, sizeof(joint_ns), "atlas_controller/gains/%s/",
             this->joints[i]->GetName().c_str());
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
    // store these directly on altasState, more efficient for pub later
    this->atlasState.kp_position[i]  =  p_val;
    this->atlasState.ki_position[i]  =  i_val;
    this->atlasState.kd_position[i]  =  d_val;
    this->atlasState.i_effort_min[i] = -i_clamp_val;
    this->atlasState.i_effort_max[i] =  i_clamp_val;
    // default k_effort is set to 1, controller relies on PID.
    this->atlasState.k_effort[i] = 255;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetExperimentalDampingPID(
  const atlas_msgs::Test::ConstPtr &_msg)
{
  if (_msg->damping.size() == this->joints.size())
    for (unsigned int i = 0; i < this->joints.size(); ++i)
      this->joints[i]->SetDamping(0, _msg->damping[i]);
  else
    ROS_DEBUG("joint test message contains different number of"
      " elements damping[%ld] than expected[%ld]",
      _msg->damping.size(), this->joints.size());

  boost::mutex::scoped_lock lock(this->mutex);

  if (_msg->kp_position.size() == this->atlasState.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->atlasState.kp_position.size());

  if (_msg->ki_position.size() == this->atlasState.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->atlasState.ki_position.size());

  if (_msg->kd_position.size() == this->atlasState.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->atlasState.kd_position.size());

  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->atlasState.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->atlasState.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->atlasState.i_effort_max.size());

  if (_msg->k_effort.size() == this->atlasState.k_effort.size())
    std::copy(_msg->k_effort.begin(), _msg->k_effort.end(),
      this->atlasState.k_effort.begin());
  else
    ROS_DEBUG("Test message contains different number of"
      " elements k_effort[%ld] than expected[%ld]",
      _msg->k_effort.size(), this->atlasState.k_effort.size());
}

////////////////////////////////////////////////////////////////////////////////
// AtlasSimInterface:
// subscribe to a control_mode string message, current valid commands are:
//   Walk, Stand, Freeze, StandPrep, User
// the command is passed to the AtlasSimInterface library.
void AtlasPlugin::OnRobotMode(const std_msgs::String::ConstPtr &_mode)
{
  // to make it Stand
  //  * StandPrep:  puts robot in standing pose while harnessed
  //  * remove the harness
  //  * after robot hits ground, switch over to Stand mode
  //  * robot should dynamically balance itself

  boost::mutex::scoped_lock lock(this->asiMutex);

  // simple state machine here to do something
  if (_mode->data == "Freeze" || _mode->data == "StandPrep" ||
      _mode->data == "Stand" || _mode->data == "Walk")
  {
    // start AtlasSimLibrary controller
    // this mode resets the timer, and automatically goes into Stand mode
    // after
    ROS_WARN("controllign AtlasSimInteface library over /atlas/control_mode "
             "is deprecated, please switch to uisng "
             "ROS topic /atlas/atlas_sim_interface_command and look "
             "for feedback on /atlas/atlas_sim_interface_state.");

    if (_mode->data == "Freeze")
    {
      this->asiState.desired_behavior =
        atlas_msgs::AtlasSimInterfaceCommand::FREEZE;
    }
    else if (_mode->data == "StandPrep")
    {
      this->asiState.desired_behavior =
        atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP;
    }
    else if (_mode->data == "Stand")
    {
      this->asiState.desired_behavior =
        atlas_msgs::AtlasSimInterfaceCommand::STAND;

    }
    else if (_mode->data == "Walk")
    {
      this->asiState.desired_behavior =
        atlas_msgs::AtlasSimInterfaceCommand::WALK;
      this->atlasControlInput.step_params.use_demo_walk = true;
    }

    this->asiState.error_code =
      this->atlasSimInterface->set_desired_behavior(_mode->data);
    if (this->asiState.error_code == NO_ERRORS)
      ROS_INFO("AtlasSimInterface: %s mode fine.", _mode->data.c_str());
    else
      ROS_INFO("AtlasSimInterface: %s mode faile with code (%d).",
               _mode->data.c_str(),
               this->asiState.error_code);

    this->ZeroAtlasCommand();

    // initialize Walk data
    if (_mode->data == "Walk")
    {
      AtlasBehaviorWalkParams *walkParams =
        &this->atlasControlInput.walk_params;
      walkParams->use_demo_walk = false;

      static const double strideSagittal = 0.20;
      static const double stepDuration = 0.63;
      static const double stepWidth = 0.12;
      for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS; ++stepId)
      {
        int isRight = stepId % 2;
        walkParams->step_data[stepId].step_index = stepId + 1;
        walkParams->step_data[stepId].foot_index = (unsigned int)isRight;
        walkParams->step_data[stepId].duration = stepDuration;
        double stepX = static_cast<double>(stepId + 1)*strideSagittal;
        double stepY = stepWidth;
        if (isRight)
          walkParams->step_data[stepId].position = AtlasVec3f(stepX, -stepY, 0);
        else
          walkParams->step_data[stepId].position = AtlasVec3f(stepX, stepY, 0);
        walkParams->step_data[stepId].yaw = 0;
      }
    }
  }
  else if (_mode->data == "User")
  {
    // revert to PID control
    this->LoadPIDGainsFromParameter();
    this->asiState.desired_behavior =
      atlas_msgs::AtlasSimInterfaceCommand::USER;
    this->atlasSimInterface->set_desired_behavior("User");
    // clear out forces
    for (unsigned i = 0; i < this->jointNames.size(); ++i)
      this->atlasControlOutput.f_out[i] = 0;
  }
  else if (_mode->data == "ragdoll")
  {
    // revert to PID control
    this->ZeroAtlasCommand();
    this->asiState.desired_behavior =
      atlas_msgs::AtlasSimInterfaceCommand::USER;
    this->atlasSimInterface->set_desired_behavior("User");
    // clear out forces
    for (unsigned i = 0; i < this->jointNames.size(); ++i)
      this->atlasControlOutput.f_out[i] = 0;
  }
  else
  {
    ROS_WARN("Unknown robot mode [%s]", _mode->data.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::GetIMUState(const common::Time &_curTime)
{
  if (this->imuSensor)
  {
    // AtlasSimInterface: populate imu in atlasRobotState
    this->atlasRobotState.imu.imu_timestamp =
      1.0e6  * _curTime.sec +
      1.0e-3 * _curTime.nsec;

    // publish separate /atlas/imu topic, to be deprecated
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = this->imuLinkName;
    imuMsg.header.stamp = ros::Time(_curTime.Double());

    // compute angular rates
    {
      math::Vector3 wLocal = this->imuSensor->GetAngularVelocity();
      this->atlasState.angular_velocity.x = wLocal.x;
      this->atlasState.angular_velocity.y = wLocal.y;
      this->atlasState.angular_velocity.z = wLocal.z;

      // publish separate /atlas/imu topic, to be deprecated
      imuMsg.angular_velocity.x = wLocal.x;
      imuMsg.angular_velocity.y = wLocal.y;
      imuMsg.angular_velocity.z = wLocal.z;

      // AtlasSimInterface: populate imu in atlasRobotState
      this->atlasRobotState.imu.angular_velocity.n[0] = wLocal.x;
      this->atlasRobotState.imu.angular_velocity.n[1] = wLocal.y;
      this->atlasRobotState.imu.angular_velocity.n[2] = wLocal.z;
    }

    // compute acceleration
    {
      math::Vector3 accel = this->imuSensor->GetLinearAcceleration();
      this->atlasState.linear_acceleration.x = accel.x;
      this->atlasState.linear_acceleration.y = accel.y;
      this->atlasState.linear_acceleration.z = accel.z;

      // publish separate /atlas/imu topic, to be deprecated
      imuMsg.linear_acceleration.x = accel.x;
      imuMsg.linear_acceleration.y = accel.y;
      imuMsg.linear_acceleration.z = accel.z;

      // AtlasSimInterface: populate imu in atlasRobotState
      this->atlasRobotState.imu.linear_acceleration.n[0] = accel.x;
      this->atlasRobotState.imu.linear_acceleration.n[1] = accel.y;
      this->atlasRobotState.imu.linear_acceleration.n[2] = accel.z;
    }

    // compute orientation
    {
      math::Quaternion imuRot = this->imuSensor->GetOrientation();
      this->atlasState.orientation.x = imuRot.x;
      this->atlasState.orientation.y = imuRot.y;
      this->atlasState.orientation.z = imuRot.z;
      this->atlasState.orientation.w = imuRot.w;

      // publish separate /atlas/imu topic, to be deprecated
      imuMsg.orientation.x = imuRot.x;
      imuMsg.orientation.y = imuRot.y;
      imuMsg.orientation.z = imuRot.z;
      imuMsg.orientation.w = imuRot.w;

      // AtlasSimInterface: populate imu in atlasRobotState
      this->atlasRobotState.imu.orientation_estimate.m_qw = imuRot.w;
      this->atlasRobotState.imu.orientation_estimate.m_qx = imuRot.x;
      this->atlasRobotState.imu.orientation_estimate.m_qy = imuRot.y;
      this->atlasRobotState.imu.orientation_estimate.m_qz = imuRot.z;
    }

    // publish separate /atlas/imu topic, to be deprecated
    this->pubImuQueue->push(imuMsg, this->pubImu);
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::GetForceTorqueSensorState(const common::Time &_curTime)
{

    // publish separate /atlas/force_torque_sensors topic, to be deprecated
    atlas_msgs::ForceTorqueSensors forceTorqueSensorsMsg;
    // publish separate /atlas/force_torque_sensors topic, to be deprecated
    forceTorqueSensorsMsg.header.stamp =
      ros::Time(_curTime.sec, _curTime.nsec);

    // get force torque at left ankle and publish
    if (this->lAnkleJoint)
    {
      physics::JointWrench wrench = this->lAnkleJoint->GetForceTorque(0u);
      this->atlasState.l_foot.force.z = wrench.body2Force.z;
      this->atlasState.l_foot.torque.x = wrench.body2Torque.x;
      this->atlasState.l_foot.torque.y = wrench.body2Torque.y;

      // publish separate /atlas/force_torque_sensors topic, to be deprecated
      forceTorqueSensorsMsg.l_foot.force.z = wrench.body2Force.z;
      forceTorqueSensorsMsg.l_foot.torque.x = wrench.body2Torque.x;
      forceTorqueSensorsMsg.l_foot.torque.y = wrench.body2Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in atlasRobotState
      this->atlasRobotState.foot_sensors[0].fz = wrench.body1Force.z;
      this->atlasRobotState.foot_sensors[0].mx = wrench.body1Torque.x;
      this->atlasRobotState.foot_sensors[0].my = wrench.body1Torque.y;
    }

    // get force torque at right ankle and publish
    if (this->rAnkleJoint)
    {
      physics::JointWrench wrench = this->rAnkleJoint->GetForceTorque(0u);
      this->atlasState.r_foot.force.z = wrench.body2Force.z;
      this->atlasState.r_foot.torque.x = wrench.body2Torque.x;
      this->atlasState.r_foot.torque.y = wrench.body2Torque.y;

      // publish separate /atlas/force_torque_sensors topic, to be deprecated
      forceTorqueSensorsMsg.r_foot.force.z = wrench.body2Force.z;
      forceTorqueSensorsMsg.r_foot.torque.x = wrench.body2Torque.x;
      forceTorqueSensorsMsg.r_foot.torque.y = wrench.body2Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in atlasRobotState
      this->atlasRobotState.foot_sensors[1].fz = wrench.body1Force.z;
      this->atlasRobotState.foot_sensors[1].mx = wrench.body1Torque.x;
      this->atlasRobotState.foot_sensors[1].my = wrench.body1Torque.y;
    }

    // get force torque at left wrist and publish
    if (this->lWristJoint)
    {
      physics::JointWrench wrench = this->lWristJoint->GetForceTorque(0u);
      this->atlasState.l_hand.force.x = wrench.body2Force.x;
      this->atlasState.l_hand.force.y = wrench.body2Force.y;
      this->atlasState.l_hand.force.z = wrench.body2Force.z;
      this->atlasState.l_hand.torque.x = wrench.body2Torque.x;
      this->atlasState.l_hand.torque.y = wrench.body2Torque.y;
      this->atlasState.l_hand.torque.z = wrench.body2Torque.z;

      // publish separate /atlas/force_torque_sensors topic, to be deprecated
      forceTorqueSensorsMsg.l_hand.force.x = wrench.body2Force.x;
      forceTorqueSensorsMsg.l_hand.force.y = wrench.body2Force.y;
      forceTorqueSensorsMsg.l_hand.force.z = wrench.body2Force.z;
      forceTorqueSensorsMsg.l_hand.torque.x = wrench.body2Torque.x;
      forceTorqueSensorsMsg.l_hand.torque.y = wrench.body2Torque.y;
      forceTorqueSensorsMsg.l_hand.torque.z = wrench.body2Torque.z;

      // AtlasSimInterface: populate wrist force torque sensor in atlasRobotState
      this->atlasRobotState.wrist_sensors[0].f.n[0] = wrench.body1Force.x;
      this->atlasRobotState.wrist_sensors[0].f.n[1] = wrench.body1Force.y;
      this->atlasRobotState.wrist_sensors[0].f.n[2] = wrench.body1Force.z;
      this->atlasRobotState.wrist_sensors[0].m.n[0] = wrench.body1Torque.x;
      this->atlasRobotState.wrist_sensors[0].m.n[1] = wrench.body1Torque.y;
      this->atlasRobotState.wrist_sensors[0].m.n[2] = wrench.body1Torque.z;
    }

    // get force torque at right wrist and publish
    if (this->rWristJoint)
    {
      physics::JointWrench wrench = this->rWristJoint->GetForceTorque(0u);
      this->atlasState.r_hand.force.x = wrench.body2Force.x;
      this->atlasState.r_hand.force.y = wrench.body2Force.y;
      this->atlasState.r_hand.force.z = wrench.body2Force.z;
      this->atlasState.r_hand.torque.x = wrench.body2Torque.x;
      this->atlasState.r_hand.torque.y = wrench.body2Torque.y;
      this->atlasState.r_hand.torque.z = wrench.body2Torque.z;

      // publish separate /atlas/force_torque_sensors topic, to be deprecated
      forceTorqueSensorsMsg.r_hand.force.x = wrench.body2Force.x;
      forceTorqueSensorsMsg.r_hand.force.y = wrench.body2Force.y;
      forceTorqueSensorsMsg.r_hand.force.z = wrench.body2Force.z;
      forceTorqueSensorsMsg.r_hand.torque.x = wrench.body2Torque.x;
      forceTorqueSensorsMsg.r_hand.torque.y = wrench.body2Torque.y;
      forceTorqueSensorsMsg.r_hand.torque.z = wrench.body2Torque.z;

      // AtlasSimInterface: populate wrist force torque sensor in atlasRobotState
      this->atlasRobotState.wrist_sensors[1].f.n[0] = wrench.body1Force.x;
      this->atlasRobotState.wrist_sensors[1].f.n[1] = wrench.body1Force.y;
      this->atlasRobotState.wrist_sensors[1].f.n[2] = wrench.body1Force.z;
      this->atlasRobotState.wrist_sensors[1].m.n[0] = wrench.body1Torque.x;
      this->atlasRobotState.wrist_sensors[1].m.n[1] = wrench.body1Torque.y;
      this->atlasRobotState.wrist_sensors[1].m.n[2] = wrench.body1Torque.z;
    }
    // publish separate /atlas/force_torque_sensors topic, to be deprecated
    this->pubForceTorqueSensorsQueue->push(forceTorqueSensorsMsg,
      this->pubForceTorqueSensors);
}

////////////////////////////////////////////////////////////////////////////////
std::string AtlasPlugin::GetBehavior(int _behavior)
{
  switch(_behavior)
  {
    case atlas_msgs::AtlasSimInterfaceCommand::USER:
      return "User";
    case atlas_msgs::AtlasSimInterfaceCommand::STAND:
      return "Stand";
    case atlas_msgs::AtlasSimInterfaceCommand::WALK:
      return "Walk";
    case atlas_msgs::AtlasSimInterfaceCommand::STEP:
      return "Step";
    case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:
      return "Manipulate";
    default:
      return std::string();
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::AtlasControlOutputToAtlasSimInterfaceState(
          atlas_msgs::AtlasBehaviorFeedback *_fb,
          AtlasBehaviorFeedback *_fbOut)
{
  _fb->status_flags = _fbOut->status_flags;
  _fb->trans_from_behavior_index = _fbOut->trans_from_behavior_index;
  _fb->trans_to_behavior_index = _fbOut->trans_to_behavior_index;
  _fb->stand_feedback.status_flags = _fbOut->stand_feedback.status_flags;
  _fb->step_feedback.status_flags = _fbOut->step_feedback.status_flags;
  _fb->walk_feedback.t_step_rem = _fb->walk_feedback.t_step_rem;
  _fb->walk_feedback.current_step_index =
    _fbOut->walk_feedback.current_step_index;
  _fb->walk_feedback.next_step_index_needed =
    _fbOut->walk_feedback.next_step_index_needed;
  _fb->walk_feedback.status_flags = _fbOut->walk_feedback.status_flags;
  for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
  {
    _fb->walk_feedback.step_data_saturated[i].step_index =
      _fbOut->walk_feedback.step_data_saturated[i].step_index;
    _fb->walk_feedback.step_data_saturated[i].foot_index =
      _fbOut->walk_feedback.step_data_saturated[i].foot_index;
    _fb->walk_feedback.step_data_saturated[i].duration =
      _fbOut->walk_feedback.step_data_saturated[i].duration;
    _fb->walk_feedback.step_data_saturated[i].pose.position =
      this->ToPoint(
      _fbOut->walk_feedback.step_data_saturated[i].position);
    _fb->walk_feedback.step_data_saturated[i].pose.orientation =
      this->ToQ(math::Quaternion(0, 0,
      _fbOut->walk_feedback.step_data_saturated[i].yaw));
      // \TODO: further rotate rot based on normal
      // sd->pose.rot = sdOut->normal ...;

    _fb->walk_feedback.step_data_saturated[i].swing_height =
      _fbOut->walk_feedback.step_data_saturated[i].swing_height;
  }
  _fb->manipulate_feedback.status_flags =
    _fbOut->manipulate_feedback.status_flags;
  _fb->manipulate_feedback.clamped.pelvis_height =
    _fbOut->manipulate_feedback.clamped.pelvis_height;
  _fb->manipulate_feedback.clamped.pelvis_yaw =
    _fbOut->manipulate_feedback.clamped.pelvis_yaw;
  _fb->manipulate_feedback.clamped.pelvis_lat =
    _fbOut->manipulate_feedback.clamped.pelvis_lat;
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}
