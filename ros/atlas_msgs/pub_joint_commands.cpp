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
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>

ros::Publisher pub_joint_commands_;
atlas_msgs::AtlasCommand ac;
std::vector<std::string> jointNames;

void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    ac.header.stamp = _js->header.stamp;

    // assign arbitrary joint angle targets
    for (unsigned int i = 0; i < jointNames.size(); i++)
      ac.position[i] = 3.2* sin((ros::Time::now() - startTime).toSec());

    pub_joint_commands_.publish(ac);
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
  jointNames.push_back("atlas::back_lbz");
  jointNames.push_back("atlas::back_mby");
  jointNames.push_back("atlas::back_ubx");
  jointNames.push_back("atlas::neck_ay");
  jointNames.push_back("atlas::l_leg_uhz");
  jointNames.push_back("atlas::l_leg_mhx");
  jointNames.push_back("atlas::l_leg_lhy");
  jointNames.push_back("atlas::l_leg_kny");
  jointNames.push_back("atlas::l_leg_uay");
  jointNames.push_back("atlas::l_leg_lax");
  jointNames.push_back("atlas::r_leg_uhz");
  jointNames.push_back("atlas::r_leg_mhx");
  jointNames.push_back("atlas::r_leg_lhy");
  jointNames.push_back("atlas::r_leg_kny");
  jointNames.push_back("atlas::r_leg_uay");
  jointNames.push_back("atlas::r_leg_lax");
  jointNames.push_back("atlas::l_arm_usy");
  jointNames.push_back("atlas::l_arm_shx");
  jointNames.push_back("atlas::l_arm_ely");
  jointNames.push_back("atlas::l_arm_elx");
  jointNames.push_back("atlas::l_arm_uwy");
  jointNames.push_back("atlas::l_arm_mwx");
  jointNames.push_back("atlas::r_arm_usy");
  jointNames.push_back("atlas::r_arm_shx");
  jointNames.push_back("atlas::r_arm_ely");
  jointNames.push_back("atlas::r_arm_elx");
  jointNames.push_back("atlas::r_arm_uwy");
  jointNames.push_back("atlas::r_arm_mwx");

  unsigned int n = jointNames.size();
  ac.position.resize(n);
  ac.velocity.resize(n);
  ac.effort.resize(n);
  ac.kp_position.resize(n);
  ac.ki_position.resize(n);
  ac.kd_position.resize(n);
  ac.kp_velocity.resize(n);
  ac.i_effort_min.resize(n);
  ac.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointNames[i], boost::is_any_of(":"));

    double val;
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", val);
    ac.kp_position[i] = val;

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", val);
    ac.ki_position[i] = val;

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", val);
    ac.kd_position[i] = val;

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val);
    ac.i_effort_min[i] = val;
    ac.i_effort_min[i] = -ac.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val);
    ac.i_effort_max[i] = val;

    ac.velocity[i]     = 0;
    ac.effort[i]       = 0;
    ac.kp_velocity[i]  = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions atlasStateSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasState>(
    "/atlas/atlas_state", 1, SetAtlasState,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  atlasStateSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  ros::Subscriber subAtlasState = rosnode->subscribe(atlasStateSo);
  // ros::Subscriber subAtlasState =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetAtlasState);

  pub_joint_commands_ =
    rosnode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
