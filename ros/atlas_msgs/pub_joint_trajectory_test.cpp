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

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo/math/Quaternion.hh>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_trajectory_test");

  bool wait = true;
  while (wait)
  {
    ros::Time t = ros::Time::now();
    if (t.toSec() > 0)
      wait = false;
  }

  ros::NodeHandle rosnode;
  ros::Publisher pub_ = rosnode.advertise<trajectory_msgs::JointTrajectory>(
    "joint_trajectory", 1, true);

  trajectory_msgs::JointTrajectory jt;

  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "atlas::pelvis";

  jt.joint_names.push_back("atlas::back_lbz");
  jt.joint_names.push_back("atlas::back_mby");
  jt.joint_names.push_back("atlas::back_ubx");
  jt.joint_names.push_back("atlas::neck_ay");
  jt.joint_names.push_back("atlas::l_leg_uhz");
  jt.joint_names.push_back("atlas::l_leg_mhx");
  jt.joint_names.push_back("atlas::l_leg_lhy");
  jt.joint_names.push_back("atlas::l_leg_kny");
  jt.joint_names.push_back("atlas::l_leg_uay");
  jt.joint_names.push_back("atlas::l_leg_lax");

  int n = 500;
  double dt = 0.01;
  double rps = 0.05;
  jt.points.resize(n);
  for (int i = 0; i < n; i++)
  {
    double theta = rps*2.0*M_PI*i*dt;
    double x1 = -0.5*sin(2*theta);
    double x2 =  0.5*sin(1*theta);
    jt.points[i].positions.clear();
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);

    // set duration
    jt.points[i].time_from_start = ros::Duration(dt);
    ROS_DEBUG("test: angles[%d][%f, %f]", n, x1, x2);
  }

  pub_.publish(jt);  // use publisher
  ros::spin();

  return 0;
}
