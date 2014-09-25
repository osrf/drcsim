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
#include <string>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <sensor_msgs/JointState.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_plugins/PubQueue.h>

#include <handle_msgs/HandleSensors.h>
#include <handle_msgs/HandleControl.h>

class IRobotHandPlugin : public gazebo::ModelPlugin
{
  /// \brief Constructor
  public: IRobotHandPlugin();

  /// \brief Destructor
  public: virtual ~IRobotHandPlugin();

  /// \brief Load the controller
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief ROS NodeHanle
  private: ros::NodeHandle* rosNode;

  /// \brief ROS callback queue
  private: ros::CallbackQueue rosQueue;

  /// \brief ROS callback queue thread
  private: void RosQueueThread();

  /// \brief ROS callback queue thread
  private: boost::thread callbackQueeuThread;

  /// \brief for publishing joint states (rviz visualization)
  private: ros::Publisher pubJointStates;
  private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

  /// \brief for publishing joint states (rviz visualization)
  private: sensor_msgs::JointState jointStates;

  // ros publish multi queue, prevents publish() blocking
  private: PubMultiQueue pmq;

  /// \brief ros topic callback to update iRobot Hand Control Commands
  /// \param[in] _msg Incoming ros message
  private: void SetHandleCommand(
    const handle_msgs::HandleControl::ConstPtr &_msg);

  /// \brief ROS control interface
  private: ros::Subscriber subHandleCommand;

  /// \brief HandleControl message (published by user)
  private: handle_msgs::HandleControl handleCommand;

  /// \brief gazebo world update connection
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief keep track of controller update sim-time
  private: gazebo::common::Time lastControllerUpdateTime;

  /// \brief iRobot Hand State
  private: handle_msgs::HandleSensors handleState;

  /// \brief Controller update mutex
  private: boost::mutex controlMutex;

  /// \brief internal pid control
  private: class ErrorTerms
    {
      double q_p;
      double d_q_p_dt;
      double q_i;
      friend class IRobotHandPlugin;
    };

  /// \brief internal pid control
  private: std::vector<ErrorTerms> errorTerms;

  /// \brief Update PID Joint Servo Controllers
  /// \param[in] _dt time step size since last update
  private: void UpdatePIDControl(double _dt);

  /// \brief Publish iRobot Hand state
  private: void GetAndPublishHandleState(const gazebo::common::Time &_curTime);

  /// \brief ROS publisher for iRobot Hand state.
  private: ros::Publisher pubHandleState;

  /// \brief ROS publisher queue for iRobot Hand state.
  private: PubQueue<handle_msgs::HandleSensors>::Ptr pubHandleStateQueue;

  /// \brief Update the controller
  private: void UpdateStates();

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

  /// \brief Convert HandleControl message values to Joint angles
  /// \param[in] _value handle_msgs::HandleControl::value[0-2], representing
  /// internal motor joint angle in radians, to be converted to
  /// tendon length, and subsequently converted to combined joint angle
  /// for baseJoint and flexureFlexJoint joints.
  /// \return _angle combined target joint angle for combined joint angle for
  /// baseJoint and flexureFlexJoint joints.
  private: double HandleControlFlexValueToFlexJointAngle(int _value);

  /// \brief Convert HandleControl message values to Joint angles
  /// \param[in] _value handle_msgs::HandleControl::value[4], representing
  /// internal motor joint angle in radians, to be converted to
  /// baseRotationJoint joint angle.
  /// \return _angle desired baseRotationJoint angle.
  private: double HandleControlSpreadValueToSpreadJointAngle(int _value);

  private: gazebo::physics::WorldPtr world;
  private: gazebo::physics::ModelPtr model;
  private: sdf::ElementPtr sdf;
  private: std::string side;

  /// \brief vector of 3, one for each finger (2 index, 1 thumb).
  private: gazebo::physics::Joint_V fingerBaseJoints;

  /// \brief vector of 2, one for each index finger.
  private: gazebo::physics::Joint_V fingerBaseRotationJoints;

  /// \brief numver of flexure twist joints * 3 (1 for each finger).
  private: std::vector<gazebo::physics::Joint_V> flexureTwistJoints;

  /// \brief numver of flexure flex joints * 3 (1 for each finger).
  private: std::vector<gazebo::physics::Joint_V> flexureFlexJoints;

  /// \brief control angle for the thumb antagonist dof.
  private: double thumbAntagonistAngle;

  /// \brief save thumb upper limit as we change it per antagonist control
  private: double thumbUpperLimit;

  private: static const int numFingers = 3;
  private: static const int numFlexLinks = 2;

  // TODO: make these constants configurable
  private: double kp_position[5];
  private: double ki_position[5];
  private: double kd_position[5];
  private: double i_position_effort_min[5];
  private: double i_position_effort_max[5];
  private: double kp_velocity[5];
  private: double ki_velocity[5];
  private: double kd_velocity[5];
  private: double i_velocity_effort_min[5];
  private: double i_velocity_effort_max[5];
};

#endif
