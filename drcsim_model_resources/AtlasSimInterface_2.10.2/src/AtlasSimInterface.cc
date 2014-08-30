/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include "AtlasControlTypes.h"
#include "AtlasSimInterface.h"
#include "AtlasSimInterfaceTypes.h"
#include "AtlasVectorTypes.h"
#include "simbicon/Controller.h"

#ifndef HAVE_EIGEN3
#undef USE_SIMBICON
#endif

#ifdef USE_SIMBICON
Controller *simbiconController;
#else
struct ErrorTerms
{
  /// error term contributions to final control output
  double q_p;
  double d_q_p_dt;
  double k_i_q_i;  // integral term weighted by k_i
  double qd_p;
  ErrorTerms()
  {
    q_p = 0.0;
    d_q_p_dt = 0.0;
    k_i_q_i = 0.0;
    qd_p = 0.0;
  }
};
ErrorTerms errorTerms[Atlas::NUM_JOINTS];
#endif

extern "C" {

//////////////////////////////////////////////////
AtlasSimInterface* create_atlas_sim_interface()
{
  std::cerr << "\nWarning: Using Atlas Shim interface. "
    << "Atlas will be uncontrolled\n";

#ifdef USE_SIMBICON
  simbiconController = new Controller(Atlas::NUM_JOINTS);
#else
#endif
  return new AtlasSimInterface();
}

//////////////////////////////////////////////////
void destroy_atlas_sim_interface()
{
#ifdef USE_SIMBICON
  delete simbiconController;
#else
#endif
}

} // end extern "C"

//////////////////////////////////////////////////
AtlasSimInterface::AtlasSimInterface()
{
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_major()
{
  return 2;
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_minor()
{
  return 10;
}

//////////////////////////////////////////////////
int AtlasSimInterface::get_version_point()
{
  // The -1 means that this library is just a shim library and it doesn't
  // provide real functionality.
  return -1;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::process_control_input(
    const AtlasControlInput& control_input,
    const AtlasRobotState& robot_state,
    AtlasControlOutput& control_output)
{

  // Joint [0-5]: FreeJoint (6) Child body : pelvis
  // Joint [6]: back_bkz (1) Parent body: pelvis Child body : ltorso 
  // Joint [7]: l_leg_hpz (1) Parent body: pelvis Child body : l_uglut 
  // Joint [8]: r_leg_hpz (1) Parent body: pelvis Child body : r_uglut 
  // Joint [9]: back_bky (1) Parent body: ltorso Child body : mtorso 
  // Joint [10]: l_leg_hpx (1) Parent body: l_uglut Child body : l_lglut 
  // Joint [11]: r_leg_hpx (1) Parent body: r_uglut Child body : r_lglut 
  // Joint [12]: back_bkx (1) Parent body: mtorso Child body : utorso 
  // Joint [13]: l_leg_hpy (1) Parent body: l_lglut Child body : l_uleg 
  // Joint [14]: r_leg_hpy (1) Parent body: r_lglut Child body : r_uleg 
  // Joint [15]: l_arm_shy (1) Parent body: utorso Child body : l_clav 
  // Joint [16]: r_arm_shy (1) Parent body: utorso Child body : r_clav 
  // Joint [17]: l_leg_kny (1) Parent body: l_uleg Child body : l_lleg 
  // Joint [18]: r_leg_kny (1) Parent body: r_uleg Child body : r_lleg 
  // Joint [19]: l_arm_shx (1) Parent body: l_clav Child body : l_scap 
  // Joint [20]: r_arm_shx (1) Parent body: r_clav Child body : r_scap 
  // Joint [xx]: l_leg_aky (1) Parent body: l_lleg Child body : l_talus 
  // Joint [xx]: r_leg_aky (1) Parent body: r_lleg Child body : r_talus 
  // Joint [xx]: l_arm_ely (1) Parent body: l_scap Child body : l_uarm 
  // Joint [xx]: r_arm_ely (1) Parent body: r_scap Child body : r_uarm 
  // Joint [xx]: l_leg_akx (1) Parent body: l_talus Child body : l_foot 
  // Joint [xx]: r_leg_akx (1) Parent body: r_talus Child body : r_foot 
  // Joint [xx]: l_arm_elx (1) Parent body: l_uarm Child body : l_larm 
  // Joint [xx]: r_arm_elx (1) Parent body: r_uarm Child body : r_larm 
  // Joint [xx]: l_arm_wry (1) Parent body: l_larm

  // convert control_input to local vectors
  std::vector<double> q_d;
  std::vector<double> qd_d;
  std::vector<double> f_d;
  std::vector<double> k_q_p;
  std::vector<double> k_q_i;
  std::vector<double> k_qd_p;
  q_d.resize(Atlas::NUM_JOINTS);
  qd_d.resize(Atlas::NUM_JOINTS);
  f_d.resize(Atlas::NUM_JOINTS);
  k_q_p.resize(Atlas::NUM_JOINTS);
  k_q_i.resize(Atlas::NUM_JOINTS);
  k_qd_p.resize(Atlas::NUM_JOINTS);
  for (unsigned int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    q_d[i] = control_input.j[i].q_d;
    qd_d[i] = control_input.j[i].qd_d;
    f_d[i] = control_input.j[i].f_d;
    k_q_p[i] = control_input.jparams[i].k_q_p;
    k_q_i[i] = control_input.jparams[i].k_q_i;
    k_qd_p[i] = control_input.jparams[i].k_qd_p;
  }

  // control robot_state to local vectors
  std::vector<double> q;
  q.resize(Atlas::NUM_JOINTS);
  std::vector<double> qd;
  qd.resize(Atlas::NUM_JOINTS);
  double time = robot_state.t;
  for (int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    // AtlasJointState
    q[i] = robot_state.j[i].q;
    qd[i] = robot_state.j[i].qd;
    // f[i] = robot_state.j[i].f;

  }

  // AtlasIMUData
  uint64_t t = robot_state.imu.imu_timestamp;
  double imu_q[4];
  imu_q[0] = robot_state.imu.orientation_estimate.m_qw;
  imu_q[1] = robot_state.imu.orientation_estimate.m_qx;
  imu_q[2] = robot_state.imu.orientation_estimate.m_qy;
  imu_q[3] = robot_state.imu.orientation_estimate.m_qz;
  AtlasVec3f imu_wf = robot_state.imu.angular_velocity;
  AtlasVec3f imu_af = robot_state.imu.linear_acceleration;
  double imu_w[3];
  double imu_a[3];
  for (unsigned int i = 0; i < 3; ++i)
  {
    imu_w[i] = imu_wf.n[i];
    imu_a[i] = imu_af.n[i];
  }

  // AtlasFootSensor
  std::vector<float> foot_fz;
  std::vector<float> foot_mx;
  std::vector<float> foot_my;
	for (int i = 0; i < Atlas::NUM_FOOT_SENSORS; ++i)
	{
    foot_fz.push_back(robot_state.foot_sensors[i].fz);
    foot_mx.push_back(robot_state.foot_sensors[i].mx);
    foot_my.push_back(robot_state.foot_sensors[i].my);
	}

  // AtlasWristSensor
  std::vector<AtlasVec3f> wrist_f;
  std::vector<AtlasVec3f> wrist_m;
	for (int i = 0; i < Atlas::NUM_WRIST_SENSORS; ++i)
	{
    wrist_f.push_back(robot_state.wrist_sensors[i].f);
    wrist_m.push_back(robot_state.wrist_sensors[i].m);
	}

  // convert robot_state to controller usable data

  /// \TODO: compute dt from timestamps
  const double dt = 0.001;

#ifdef USE_SIMBICON
  /// \TODO: use states passed into simbicon controller
  /// to trigger state change.
  std::vector<double> torques = simbiconController->update(dt,
    Atlas::NUM_JOINTS, &q[0], &qd[0], imu_q, imu_w, imu_a,
    foot_fz[0], foot_mx[0], foot_my[0],   // left foot
    foot_fz[1], foot_mx[1], foot_my[1]);  // right foot

  // copy control torque to control_output
  for (int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    control_output.f_out[i] = torques[i];
  }
#else
  // Copied from AtlasPlugin::UpdatePIDControl and modified locally
  for (unsigned int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    /// \TODO: get joint limits from raw config or sdf?
    // truncate joint position within range of motion
    // double positionTarget = math::clamp(
    //   this->atlasCommand.position[i],
    //   this->joints[i]->GetLowStop(0).Radian(),
    //   this->joints[i]->GetHighStop(0).Radian());

    // position error
    double q_p = q_d[i] - q[i];

    // compute differential error term
    if (dt > 0.0)
      errorTerms[i].d_q_p_dt = (q_p - errorTerms[i].q_p) / dt;

    // store position error
    errorTerms[i].q_p = q_p;

    // approximate effort generated by a non-zero joint velocity state
    // this is the approximate force of the infinite bandwidth
    // kp_velocity term, we'll use this to bound additional forces later.
    double kpVelocityDampingEffort = k_qd_p[i] * qd[i];

    // compute integral error term
    errorTerms[i].k_i_q_i =
      errorTerms[i].k_i_q_i + dt * k_q_i[i] * errorTerms[i].q_p;
    /// \TODO: bound integral error by min/max?

    /// \TODO: get k_effort from AtlasSimInterface::get_behavior_joint_weights
    // k_effort is a double between 0 and 1
    // const double k_effort = 1.0;

    // compute force cmd
    const double k_q_d = 0.0; // not used in this controller.
    double forceUnclamped = k_q_p[i] * errorTerms[i].q_p +
                                       errorTerms[i].k_i_q_i +
                               k_q_d * errorTerms[i].d_q_p_dt +
                           k_qd_p[i] * qd_d[i] +
                                       f_d[i];

    /// \TODO: need effort limits to do below
    // keep unclamped force for integral tie-back calculation
    // double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i],
    //   this->effortLimit[i]);
    //
    // clamp force after integral tie-back
    // shift by kpVelocityDampingEffort to prevent controller from
    // exerting too much force from use of kp_velocity --> cfm damping
    // pass through.
    // double forceClamped = math::clamp(forceUnclamped,
    //   -this->effortLimit[i] + kpVelocityDampingEffort,
    //    this->effortLimit[i] + kpVelocityDampingEffort);

    // force to be applied to joint
    // fill in control output command efforts.
    control_output.f_out[i] = forceUnclamped;
  }
#endif

  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::reset_control()
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::set_desired_behavior(
    const std::string& behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_desired_behavior(
    std::string& desired_behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_current_behavior(
    std::string& current_behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_num_behaviors(int& num_behaviors)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_behavior_at_index(int index,
    std::string& behavior)
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_behavior_joint_weights(
    const std::string& behavior,
    float joint_control_weights[NUM_JOINTS])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_current_behavior_joint_weights(
    float joint_control_weights[NUM_JOINTS])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterface::get_estimated_position(
    AtlasPositionData& robot_pos_est,
    AtlasVec3f foot_pos_est[Atlas::NUM_FEET])
{
  return AtlasSim::NO_ERRORS;
}

//////////////////////////////////////////////////
std::string AtlasSimInterface::get_error_code_text(AtlasErrorCode ec)
{
  return std::string("NO ERRORS");
}
