/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "physics/physics.h"
#include "transport/transport.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Events.hh"

int main(int argc, char** argv)
{
  // std::list<std::string> worldNames;
  // gazebo::transport::get_topic_namespaces(worldNames);

  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("pose_pub_node");

  gazebo::transport::PublisherPtr pub;
  pub.reset();
  pub = node->Advertise<gazebo::msgs::PoseTrajectory>("/model_poses",10);

  gazebo::msgs::PoseTrajectory pose_trajectory;

  gazebo::msgs::PoseStamped pose_stamped;
  pose_stamped.mutable_time()->set_sec(1);
  pose_stamped.mutable_time()->set_nsec(0);
  pose_stamped.mutable_pose()->set_name("test_pose_stamped");
  pose_stamped.mutable_pose()->mutable_position()->set_x(0);
  pose_stamped.mutable_pose()->mutable_position()->set_y(0);
  pose_stamped.mutable_pose()->mutable_position()->set_z(1.5);
  pose_stamped.mutable_pose()->mutable_orientation()->set_w(1);
  pose_stamped.mutable_pose()->mutable_orientation()->set_x(0);
  pose_stamped.mutable_pose()->mutable_orientation()->set_y(0);
  pose_stamped.mutable_pose()->mutable_orientation()->set_z(0);

  pose_trajectory.set_name("test_trajecotry");
  pose_trajectory.set_id(100);
  pose_trajectory.add_pose_stamped()->CopyFrom(pose_stamped);

  pub->Publish(pose_trajectory);
  printf("done publishing message\n");

  return 0;
}
