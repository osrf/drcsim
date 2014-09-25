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
#ifndef ACTIONLIB_SERVER_H
#define ACTIONLIB_SERVER_H

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <atlas_msgs/WalkDemoAction.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <actionlib/server/simple_action_server.h>

#define NUM_REQUIRED_WALK_STEPS 4

// ACTIONLIB_SERVER_H
#endif

// actionlib simple action server
typedef actionlib::SimpleActionServer<atlas_msgs::WalkDemoAction> ActionServer;



class ASIActionServer
{
  /// \brief constructor
  public: ASIActionServer();

  /// \brief actionlib simple action server executor callback
  public: void ActionServerCallback();

  /// \brief Subscriber callback for BDI_interface topic
  public: void BDIStateCallback(
    const atlas_msgs::AtlasSimInterfaceState::ConstPtr &_msg);

  /// \brief Aborts current goal
  public: void abortGoal(std::string reason);

  /// \brief Subscriber to AtlasState topic
  private: void AtlasStateCB(const atlas_msgs::AtlasState::ConstPtr &_msg);

  /// \brief action server callback
  private: void ActionServerCB();

  /// \brief action server callback
  private: void ASIStateCB(
      const atlas_msgs::AtlasSimInterfaceState::ConstPtr &_msg);

  /// \brief Transforms a step pose based on current estimated robot world pose
  public: void transformStepPose(geometry_msgs::Pose &_pose);

  /// \brief Current robot position, pulled from ASIStateCB
  private: geometry_msgs::Vector3 robotPosition;

  /// \brief Orientation of the robot, pulled from AtlasState's IMU
  private: tf::Quaternion robotOrientation;

  /// \brief lock while updating control modes
  private: boost::mutex actionServerMutex;

  /// \brief lock while updating robot state
  private: boost::mutex robotStateMutex;

  /// \brief ROS action server
  private: ActionServer *actionServer;

  /// \brief AtlasSimInterfaceState subscriber
  private: ros::Subscriber ASIStateSubscriber;

  /// TODO use this subscriber to get where the feet are located
  /// \brief AtlasState subscriber
  private: ros::Subscriber atlasStateSubscriber;

  /// \brief AtlasSimInterfaceCommand Publisher
  private: ros::Publisher atlasCommandPublisher;

  /// \brief local copy of the goal
  private: atlas_msgs::WalkDemoGoal activeGoal;

  /// \brief actionlib feedback
  private: atlas_msgs::WalkDemoFeedback actionServerFeedback;

  /// \brief actionlib result
  private: atlas_msgs::WalkDemoResult actionServerResult;

  /// \brief used for trajectory rollout
  private: std::vector<atlas_msgs::AtlasBehaviorStepData>
  stepTrajectory;

  /// \brief ROS node handle
  private: ros::NodeHandle rosNode;

  /// \brief bool to check if a goal is being executed, otherwise send stand
  private: bool executingGoal;

  /// \brief used to determine if a new goal exists, during which conflicting
  /// information is relayed through atlas_sim_interface_state
  private: bool newGoal;

  /// \brief internal state variable
  private: bool isStepping;

  /// \brief internal state variable
  private: int pubCount;

  /// \brief Keeps track of the current step in process when walking
  private: unsigned int currentStepIndex;
};
