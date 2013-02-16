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
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/common/Time.hh>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointCommands;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    jointCommands.header.stamp = _js->header.stamp;

    // assign arbitrary joint angle targets
    for (unsigned int i = 0; i < jointCommands.name.size(); i++)
      jointCommands.position[i] =
        3.2* sin((ros::Time::now() - startTime).toSec());

    pub_joint_commands_.publish(jointCommands);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_command_test");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // must match those inside AtlasPlugin
  jointCommands.name.push_back("atlas::back_lbz");
  jointCommands.name.push_back("atlas::back_mby");
  jointCommands.name.push_back("atlas::back_ubx");
  jointCommands.name.push_back("atlas::neck_ay");
  jointCommands.name.push_back("atlas::l_leg_uhz");
  jointCommands.name.push_back("atlas::l_leg_mhx");
  jointCommands.name.push_back("atlas::l_leg_lhy");
  jointCommands.name.push_back("atlas::l_leg_kny");
  jointCommands.name.push_back("atlas::l_leg_uay");
  jointCommands.name.push_back("atlas::l_leg_lax");
  jointCommands.name.push_back("atlas::r_leg_uhz");
  jointCommands.name.push_back("atlas::r_leg_mhx");
  jointCommands.name.push_back("atlas::r_leg_lhy");
  jointCommands.name.push_back("atlas::r_leg_kny");
  jointCommands.name.push_back("atlas::r_leg_uay");
  jointCommands.name.push_back("atlas::r_leg_lax");
  jointCommands.name.push_back("atlas::l_arm_usy");
  jointCommands.name.push_back("atlas::l_arm_shx");
  jointCommands.name.push_back("atlas::l_arm_ely");
  jointCommands.name.push_back("atlas::l_arm_elx");
  jointCommands.name.push_back("atlas::l_arm_uwy");
  jointCommands.name.push_back("atlas::l_arm_mwx");
  jointCommands.name.push_back("atlas::r_arm_usy");
  jointCommands.name.push_back("atlas::r_arm_shx");
  jointCommands.name.push_back("atlas::r_arm_ely");
  jointCommands.name.push_back("atlas::r_arm_elx");
  jointCommands.name.push_back("atlas::r_arm_uwy");
  jointCommands.name.push_back("atlas::r_arm_mwx");

  unsigned int n = jointCommands.name.size();
  jointCommands.position.resize(n);
  jointCommands.velocity.resize(n);
  jointCommands.effort.resize(n);
  jointCommands.kp_position.resize(n);
  jointCommands.ki_position.resize(n);
  jointCommands.kd_position.resize(n);
  jointCommands.kp_velocity.resize(n);
  jointCommands.i_effort_min.resize(n);
  jointCommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointCommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointCommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointCommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointCommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointCommands.i_effort_min[i]);
    jointCommands.i_effort_min[i] = -jointCommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointCommands.i_effort_max[i]);

    jointCommands.velocity[i]     = 0;
    jointCommands.effort[i]       = 0;
    jointCommands.kp_velocity[i]  = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
