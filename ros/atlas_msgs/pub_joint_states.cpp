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
#include <pthread.h>

#include <sys/time.h>
#include <time.h>

#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>

boost::mutex mutex;
ros::Publisher pub_atlas_state;
atlas_msgs::AtlasState as;
atlas_msgs::AtlasCommand ac;
unsigned long msCount = 0;

void SetAtlasCommand(const atlas_msgs::AtlasCommand::ConstPtr &_ac)
{
  // printf("%f %ld %f\n", ros::Time::now().toSec()*1.0e3, msCount, _ac->header.stamp.toSec()*1.0e3);
  // return;
  {
    //boost::mutex::scoped_lock lock(mutex);

    // bool missed = (_ac->header.seq - ac.header.seq > 1);
      
    ac.header.stamp = _ac->header.stamp;
    // printf(" ac receive %ld %f %d", msCount, ac.header.stamp.toSec()*1.0e3, ac.header.seq);
    // if (msCount - ac.header.stamp.toSec()*1.0e3 > 3)
    //   printf(" old ");
    // if (missed)
    //   printf(" missed ");
    // printf("\n");
  }
  // pthread_yield();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_atlas_state_test");

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
  std::vector<std::string> jointNames;
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
  as.position.resize(n);
  as.velocity.resize(n);
  as.effort.resize(n);

  // ros topic subscribtions
  ros::SubscribeOptions atlasCommandSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasCommand>(
    "/atlas/joint_commands", 1, SetAtlasCommand,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  atlasCommandSo.transport_hints =
    // ros::TransportHints().reliable().tcpNoDelay(true);
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  ros::Subscriber subAtlasCommand = rosnode->subscribe(atlasCommandSo);

  // ros::Subscriber subAtlasCommand =
  //   rosnode->subscribe("/atlas/joint_commands", 1000, SetAtlasCommand);

  pub_atlas_state =
    rosnode->advertise<atlas_msgs::AtlasState>(
    "/atlas/atlas_state", 1, true);

  boost::thread spinThread(boost::bind(&ros::spin));

  as.header.stamp = ros::Time();
  ros::WallRate r(1000.0);

  static const unsigned int NNN = 100000;
  double out1[NNN];
  double out2[NNN];
  double out3[NNN];

  // double lastTime;
  while(ros::ok() && msCount < NNN)
  {
    msCount++;
    // double age;
    as.header.stamp = ros::Time(msCount/1.0e3);
    {
      //boost::mutex::scoped_lock lock(mutex);
      // age = msCount - ac.header.stamp.toSec()*1.0e3;
      // if (ac.header.stamp.toSec()*1.0e3 == lastTime)
      //   printf("dropped package \n");
      // lastTime = ac.header.stamp.toSec()*1.0e3;
    }
    // pthread_yield();

    // pub_atlas_state.publish(as);
    // pthread_yield();

    //if (age > 3.0)
    //  printf("%ld, %f, %f\n", msCount, age, ac.header.stamp.toSec()*1.0e3);

    // out1[msCount-1] = ros::Time::now().toSec()*1.0e3;
    out1[msCount-1] = ros::Time::now().toSec()*1.0e3;
    out2[msCount-1] = (double)msCount;
    out3[msCount-1] = ac.header.stamp.toSec()*1.0e3;
    // r.sleep();
    usleep(1000);
  }

  for(unsigned int i =0; i < NNN; ++i)
    printf("%f %f %f\n", out1[i], out2[i], out3[i]);

  return 0;
}
