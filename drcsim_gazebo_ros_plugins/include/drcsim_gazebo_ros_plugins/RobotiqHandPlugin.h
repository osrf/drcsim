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
#ifndef GAZEBO_ROBOTIQ_HAND_PLUGIN_HH
#define GAZEBO_ROBOTIQ_HAND_PLUGIN_HH

#include <string>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/PubQueue.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>

/// \brief A plugin that implements the Robotiq 3-Finger Adaptative Gripper.
/// The plugin exposes the next parameters via SDF tags:
///   * <side> Determines if we are controlling the left or right hand. This is
///            a required parameter and the allowed values are 'left' or 'right'
///   * <kp_position> P gain for the PID that controls the position
///                   of the joints. This parameter is optional.
///   * <ki_position> I gain for the PID that controls the position
///                   of the joints. This parameter is optional.
///   * <kd_position> D gain for the PID that controls the position
///                   of the joints. This parameter is optional.
///   * <position_effort_min> Minimum output of the PID that controls the position
///                           of the joints. This parameter is optional.
///   * <position_effort_max> Maximum output of the PID that controls the position
///                           of the joints. This parameter is optional.
///   * <topic_command> ROS topic name used to send new commands to the hand.
///                     This parameter is optional.
///   * <topic_state> ROS topic name used to receive state from the hand.
///                   This parameter is optional.
class RobotiqHandPlugin : public gazebo::ModelPlugin
{
  /// \brief Hand states.
  enum State
  {
    Disabled = 0,
    Emergency,
    ICS,
    ICF,
    ChangeModeInProgress,
    Simplified
  };

  /// \brief Different grasping modes.
  enum GraspingMode
  {
    Basic = 0,
    Wide,
    Pinch,
    Scissor
  };

  /// \brief Constructor.
  public: RobotiqHandPlugin();

  /// \brief Destructor.
  public: virtual ~RobotiqHandPlugin();

  /// \brief Load the controller.
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief ROS callback queue thread.
  private: void RosQueueThread();

  /// \brief ros topic callback to update Robotiq Hand Control Commands.
  /// \param[in] _msg Incoming ROS message with the next hand command.
  private: void SetHandleCommand(
    const robotiq_s_model_control::SModel_robot_output::ConstPtr &_msg);

  /// \brief Update PID Joint controllers.
  /// \param[in] _dt time step size since last update.
  private: void UpdatePIDControl(double _dt);

  /// \brief Publish Robotiq Hand state.
  private: void GetAndPublishHandleState();

  /// \brief Update the controller.
  private: void UpdateStates();

  /// \brief Grab pointers to all the joints we're going to use.
  /// \return true on success, false otherwise.
  private: bool FindJoints();

  /// \brief Fully open the hand at half of the maximum speed.
  private: void ReleaseHand();

  /// \brief Stop the fingers.
  private: void StopHand();

  /// \brief Checks if the hand is fully open.
  /// return True when all the fingers are fully open or false otherwise.
  private: bool IsHandFullyOpen();

  /// \brief Internal helper to reduce code duplication. If the joint name is
  /// found, a pointer to the joint is added to a vector of joint pointers.
  /// \param[in] _jointName Joint name.
  /// \param[out] _joints Vector of joint pointers.
  /// \return True when the joint was found or false otherwise.
  private: bool GetAndPushBackJoint(const std::string& _jointName,
                                    gazebo::physics::Joint_V& _joints);

  /// \brief Verify that one command field is within the correct range.
  /// \param[in] _label Label of the field. E.g.: rACT, rMOD.
  /// \param[in] _min Minimum value.
  /// \param[in] _max Maximum value.
  /// \param[in] _v Value to be verified.
  /// \return True when the value is within the limits or false otherwise.
  private: bool VerifyField(const std::string &_label, int _min,
                            int _max, int _v);

  /// \brief Verify that all the command fields are within the correct range.
  /// \param[in] _command Robot output message.
  /// \return True if all the fields are withing the correct range or false
  /// otherwise.
  private: bool VerifyCommand(
    const robotiq_s_model_control::SModel_robot_output::ConstPtr &_command);

  /// \brief Number of joints in the hand.
  /// The three fingers can do abduction/adduction.
  /// Fingers 1 and 2 can do circumduction in one axis.
  private: static const int NumJoints = 5;

  /// \brief Default topic name for sending control updates to the left hand.
  private: static const std::string DefaultLeftTopicCommand;

  /// \brief Default topic name for receiving state updates from the left hand.
  private: static const std::string DefaultLeftTopicState;

  /// \brief Default topic name for sending control updates to the right hand.
  private: static const std::string DefaultRightTopicCommand;

  /// \brief Default topic name for receiving state updates from the right hand.
  private: static const std::string DefaultRightTopicState;

  /// \brief ROS topic name for sending control updates to the hand.
  private: std::string controlTopicName;

  /// \brief ROS topic name for receiving state updates from the hand.
  private: std::string stateTopicName;

  /// \brief ROS NodeHanle.
  private: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS callback queue.
  private: ros::CallbackQueue rosQueue;

  /// \brief ROS callback queue thread.
  private: boost::thread callbackQueueThread;

  // ros publish multi queue, prevents publish() blocking
  private: PubMultiQueue pmq;

  /// \brief ROS control interface
  private: ros::Subscriber subHandleCommand;

  /// \brief HandleControl message. Originally published by user but some of the
  /// fields might be internally modified. E.g.: When releasing the hand for
  // changing the grasping mode.
  private: robotiq_s_model_control::SModel_robot_output handleCommand;

  /// \brief HandleControl message. Last command received before changing the
  /// grasping mode.
  private: robotiq_s_model_control::SModel_robot_output lastHandleCommand;

  /// \brief Original HandleControl message (published by user and unmodified).
  private: robotiq_s_model_control::SModel_robot_output userHandleCommand;

  /// \brief gazebo world update connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief keep track of controller update sim-time.
  private: gazebo::common::Time lastControllerUpdateTime;

  /// \brief Robotiq Hand State.
  private: robotiq_s_model_control::SModel_robot_input handleState;

  /// \brief Controller update mutex.
  private: boost::mutex controlMutex;

  /// \brief Grasping mode.
  private: GraspingMode graspingMode;

  /// \brief Hand state.
  private: State handState;

  /// \brief ROS publisher for Robotiq Hand state.
  private: ros::Publisher pubHandleState;

  /// \brief ROS publisher queue for Robotiq Hand state.
  private: PubQueue<robotiq_s_model_control::SModel_robot_input>::Ptr
    pubHandleStateQueue;

  /// \brief World pointer.
  private: gazebo::physics::WorldPtr world;

  /// \brief Parent model of the hand.
  private: gazebo::physics::ModelPtr model;

  /// \brief Pointer to the SDF of this plugin.
  private: sdf::ElementPtr sdf;

  /// \brief Used to select between 'left' or 'right' hand.
  private: std::string side;

  /// \brief Vector containing all the finger joints.
  private: gazebo::physics::Joint_V fingerJoints;

  /// \brief PIDs used to control the finger positions.
  private: gazebo::common::PID posePID[NumJoints];
};

#endif
