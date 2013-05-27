/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math/gzmath.hh>

#include <iostream>
#include <cstdlib> 


/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::load(_argc, _argv);

  std::string name = _argv[1];
  int id   = 0;
  double x = atof(_argv[2]);
  double y = atof(_argv[3]);
  double z = atof(_argv[4]);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Start transport
  gazebo::transport::run();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Model>("/gazebo/vrc_task_1/model/modify");

  // set the model
  gazebo::msgs::Model model_msg;
  if (name == "atlas")
      id = 335;
  else if (name == "drc_vehicle")
      id = 0;

  model_msg.set_name(name);
  model_msg.set_id(id);
  gazebo::math::Pose pose(x, y, z, 0, 0, 0);
  gazebo::msgs::Set(model_msg.mutable_pose(), pose);

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  gazebo::common::Time::MSleep(100);
  pub->Publish(model_msg);

  // Make sure to shut everything down.
  gazebo::transport::fini();
}

