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
#ifndef GAZEBO_IROBOT_HAND_PLUGIN_HH
#define GAZEBO_IROBOT_HAND_PLUGIN_HH

#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

class IRobotHandPlugin : public gazebo::ModelPlugin
{
  /// \brief Constructor
  public: IRobotHandPlugin();

  /// \brief Destructor
  public: virtual ~IRobotHandPlugin();

  /// \brief Load the controller
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Grab pointers to all the joints we're going to use.
  /// \return true on success, false otherwise
  private: bool FindJoints();

  /// \brief Set the damping and stiffness of various joints
  private: void SetJointSpringDamper();

  /// \brief Internal helper to reduce code duplication.
  private: bool GetAndPushBackJoint(const std::string& _joint_name,
                                    gazebo::physics::Joint_V& _joints);

  /// \brief Convert Kp and Kd to CFM and ERP
  /// \param[in] _dt time step size
  /// \param[in] _kp spring stiffness
  /// \param[in] _kd spring damping
  /// \param[out] _cfm equivalent constraint force mixing
  /// \param[out] _erp equivalent error reduction parameter
  private: void KpKdToCFMERP(const double _dt,
                             const double _kp, const double _kd,
                             double &_cfm, double &_erp);

  /// \brief Convert CFM and ERP to Kp and Kd
  /// \param[in] _dt time step size
  /// \param[in] _cfm constraint force mixing
  /// \param[in] _erp error reduction parameter
  /// \param[out] _kp equivalent spring stiffness
  /// \param[out] _kd equivalent spring damping
  private: void CFMERPToKpKd(const double _dt,
                             const double _cfm, const double _erp,
                             double &_kp, double &_kd);

  private: gazebo::physics::WorldPtr world;
  private: gazebo::physics::ModelPtr model;
  private: sdf::ElementPtr sdf;
  private: std::string side;
  private: gazebo::physics::Joint_V fingerBaseJoints;
  private: gazebo::physics::Joint_V fingerBaseRotationJoints;
  private: std::vector<gazebo::physics::Joint_V> fingerFlexTwistJoints;

  private: static const int numFingers = 3;
  private: static const int numFlexLinks = 8;

  // TODO: make these constants configurable
  // private: double flexJointCFM;  // = 9.0;  // test value
  // private: double flexJointERP;  // = 0.1;  // test value
  // private: double twistJointCFM;  // = 0.48;  // test value
  // private: double twistJointERP;  // = 0.05;  // test value
  // private: double baseJointCFM;  // = 9.0;  // test value
  // private: double baseJointERP;  // = 0.1;  // test value
};

#endif
