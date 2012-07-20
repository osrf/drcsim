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
#include "gazebo/gazebo.hh"
#include "physics/physics.h"
#include "transport/Node.hh"
#include "transport/Publisher.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Events.hh"

gazebo::common::Time sim_time(-1);
void OnStats( const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &_msg)
{
  sim_time = gazebo::common::Time(_msg->sim_time().sec(),_msg->sim_time().nsec());
  gzerr << "got stat [" << sim_time << "]\n";
}


int main(int argc, char** argv)
{
  gazebo::load();
  gazebo::init();

  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");

  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/world_stats", &OnStats);
  gazebo::run();
  while(sim_time < 0)
  {
    gzwarn << sim_time << "\n";
  }

  gazebo::transport::PublisherPtr pub;
  pub = node->Advertise<gazebo::msgs::PoseTrajectory>("/gazebo/model_poses",10);

  gazebo::msgs::PoseTrajectory pose_trajectory;
  pose_trajectory.set_name("test_trajecotry");
  pose_trajectory.set_id(100);

  for (int k = 0; k < 1000; k++)
  {
    gazebo::msgs::PoseStamped pose_stamped;
    gazebo::common::Time t(0.01*k);
    pose_stamped.mutable_time()->set_sec((sim_time+t).sec);
    pose_stamped.mutable_time()->set_nsec((sim_time+t).nsec);
    pose_stamped.mutable_pose()->set_name("test_pose_stamped");
    pose_stamped.mutable_pose()->mutable_position()->set_x(0.01*k);
    pose_stamped.mutable_pose()->mutable_position()->set_y(0);
    pose_stamped.mutable_pose()->mutable_position()->set_z(1.4);
    pose_stamped.mutable_pose()->mutable_orientation()->set_w(1);
    pose_stamped.mutable_pose()->mutable_orientation()->set_x(0);
    pose_stamped.mutable_pose()->mutable_orientation()->set_y(0);
    pose_stamped.mutable_pose()->mutable_orientation()->set_z(0);

    pose_trajectory.add_pose_stamped()->CopyFrom(pose_stamped);
  }

  int count = 0;
  while (count < 1)
  {
    pub->Publish(pose_trajectory);
    count++;
  }

  gazebo::run();
  gazebo::fini();
  return 0;
}
