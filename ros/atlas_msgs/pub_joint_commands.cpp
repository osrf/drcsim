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
#include <math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/common/Console.hh>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

#include <time.h>  // for timespec


ros::Publisher pub_joint_commands_;
ros::Time last_ros_time_;
ros::Time last_header_time_;
gazebo::common::Time last_wall_time_;
ros::NodeHandle* rosnode;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  struct timespec tv;
  gazebo::common::Time wall_time;

  clock_gettime(0, &tv);
  wall_time = tv;
  // printf("rt[%f] jst[%f] st[%f] dst[%f] drt[%f]\n",
  // printf("marker %f, %f, %f, %f, %f %f\n",
  //   wall_time.Double()*1000.0,
  //   _js->header.stamp.toSec()*1000.0,
  //   ros::Time::now().toSec()*1000.0,
  //   (ros::Time::now() - last_ros_time_).toSec()*1000.0,
  //   (wall_time - last_wall_time_.Double()*1000.0,
  //   (_js->header.stamp - last_header_time_).toSec()*1000.0);

  last_header_time_ = _js->header.stamp;
  last_ros_time_ = ros::Time::now();
  last_wall_time_ = wall_time;

  {
    osrf_msgs::JointCommands jc;

    jc.header.stamp = _js->header.stamp;  // for testing round trip time
    // jc.header.stamp = ros::Time::now();

    jc.name.push_back("atlas::back_lbz");
    jc.name.push_back("atlas::back_mby");
    jc.name.push_back("atlas::back_ubx");
    jc.name.push_back("atlas::neck_ay");
    jc.name.push_back("atlas::l_leg_uhz");
    jc.name.push_back("atlas::l_leg_mhx");
    jc.name.push_back("atlas::l_leg_lhy");
    jc.name.push_back("atlas::l_leg_kny");
    jc.name.push_back("atlas::l_leg_uay");
    jc.name.push_back("atlas::l_leg_lax");
    jc.name.push_back("atlas::r_leg_lax");
    jc.name.push_back("atlas::r_leg_uay");
    jc.name.push_back("atlas::r_leg_kny");
    jc.name.push_back("atlas::r_leg_lhy");
    jc.name.push_back("atlas::r_leg_mhx");
    jc.name.push_back("atlas::r_leg_uhz");
    jc.name.push_back("atlas::l_arm_elx");
    jc.name.push_back("atlas::l_arm_ely");
    jc.name.push_back("atlas::l_arm_mwx");
    jc.name.push_back("atlas::l_arm_shx");
    jc.name.push_back("atlas::l_arm_usy");
    jc.name.push_back("atlas::l_arm_uwy");
    jc.name.push_back("atlas::r_arm_elx");
    jc.name.push_back("atlas::r_arm_ely");
    jc.name.push_back("atlas::r_arm_mwx");
    jc.name.push_back("atlas::r_arm_shx");
    jc.name.push_back("atlas::r_arm_usy");
    jc.name.push_back("atlas::r_arm_uwy");

    int n = jc.name.size();
    jc.position.resize(n);
    jc.velocity.resize(n);
    jc.effort.resize(n);
    jc.kp_position.resize(n);
    jc.ki_position.resize(n);
    jc.kd_position.resize(n);
    jc.kp_velocity.resize(n);
    jc.i_effort_min.resize(n);
    jc.i_effort_max.resize(n);

    for (int i = 0; i < n; i++)
    {
      std::vector<std::string> pieces;
      boost::split(pieces, jc.name[i], boost::is_any_of(":"));

      rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
        jc.kp_position[i]);

      rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
        jc.ki_position[i]);

      rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
        jc.kd_position[i]);

      rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        jc.i_effort_min[i]);
      jc.i_effort_min[i] = -jc.i_effort_min[i];

      rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        jc.i_effort_max[i]);
      
      // turn off integral and derivative gains
      jc.ki_position[i] *= 0.0;
      jc.kd_position[i] *= 0.0;

      jc.position[i]     = 0.7* cos(ros::Time::now().toSec());
      jc.velocity[i]     = 0;
      jc.effort[i]       = 0;
      jc.kp_velocity[i]  = 0;
    }

    pub_joint_commands_.publish(jc);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_command_test");

  rosnode = new ros::NodeHandle();

  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
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
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
