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

ros::Publisher pubAtlasCommand;
atlas_msgs::AtlasCommand ac;
atlas_msgs::AtlasState as;
boost::mutex mutex;
ros::Time t0;
unsigned int numJoints = 30;
unsigned int seed = 0;

void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_as)
{
  static ros::Time startTime = ros::Time::now();
  t0 = startTime;

  // lock to copy incoming AtlasState
  {
    boost::mutex::scoped_lock lock(mutex);
    as = *_as;
  }

  // uncomment to simulate state filtering
  // usleep(1000);
}

void Work()
{
  // simulated worker thread
  while(true)
  {
    // lock to get data from AtlasState
    {
      boost::mutex::scoped_lock lock(mutex);
      // for testing round trip time
      ac.header.stamp = as.header.stamp;
    }

    // simulate working
    usleep(2000);

    // assign arbitrary joint angle targets
    for (unsigned int i = 0; i < numJoints; ++i)
    {
      double randNorm = 2.0 * static_cast<double>(rand_r(&seed)) /
        static_cast<double>(RAND_MAX) - 1.0;

      ac.position[i] = 3.2* sin(randNorm + (ros::Time::now() - t0).toSec());

      ac.k_effort[i] = 255;
    }

    // Let AtlasPlugin driver know that a response over /atlas/atlas_command
    // is expected every 5ms; and to wait for AtlasCommand if none has been
    // received yet. Use up the delay budget if wait is needed.
    ac.desired_controller_period_ms = 5;

    pubAtlasCommand.publish(ac);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_atlas_command");

  ros::NodeHandle *rosnode = new ros::NodeHandle();

  // Get atlas version, and set joint count
  int atlasVersion = 5;
  if (!rosnode->getParam("atlas_version", atlasVersion))
  {
    ROS_WARN("atlas_version not set, assuming version 5");
  }

  int atlasSubVersion = 0;
  rosnode->getParam("atlas_sub_version", atlasSubVersion);

  // Atlas version 4.1 has no wry2 joints
  if ((atlasVersion == 4 && atlasSubVersion == 0) || atlasVersion > 4)
  {
    numJoints = 30;
  }
  else
  {
    numJoints = 28;
  }

  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  ac.position.resize(numJoints);
  ac.k_effort.resize(numJoints);

  // default values for AtlasCommand
  for (unsigned int i = 0; i < numJoints; ++i)
    ac.k_effort[i] = 255;

  // ros topic subscribtions
  ros::SubscribeOptions atlasStateSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasState>(
    "/atlas/atlas_state", 100, SetAtlasState,
    ros::VoidPtr(), rosnode->getCallbackQueue());
  atlasStateSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  ros::Subscriber subAtlasState = rosnode->subscribe(atlasStateSo);

  // ros topic publisher
  pubAtlasCommand = rosnode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/atlas_command", 100, true);

  // simulated worker thread
  boost::thread work = boost::thread(&Work);

  ros::spin();

  return 0;
}
