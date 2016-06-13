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

#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>
#include <cstdlib>

# if GAZEBO_MAJOR_VERSION >= 6
#include <gazebo/gazebo_client.hh>
# endif


boost::mutex g_mutex;
boost::condition_variable g_condition;
gazebo::msgs::Request *g_requestMsg;
gazebo::msgs::Model g_modelMsg;

/////////////////////////////////////////////////
void OnResponse(ConstResponsePtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  if (!g_requestMsg || _msg->id() != g_requestMsg->id())
    return;

  if (_msg->has_type() && _msg->type() == g_modelMsg.GetTypeName())
  {
    g_modelMsg.ParseFromString(_msg->serialized_data());
    g_condition.notify_all();
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
#if GAZEBO_MAJOR_VERSION > 2
# if GAZEBO_MAJOR_VERSION >= 6
  gazebo::client::setup(_argc, _argv);
# else
  gazebo::setupClient(_argc, _argv);
# endif
#else
  gazebo::load(_argc, _argv);
#endif

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

  // Create request for entity_info
  gazebo::transport::PublisherPtr requestPub =
    node->Advertise<gazebo::msgs::Request>("~/request");
  g_requestMsg = gazebo::msgs::CreateRequest("entity_info", name);

  {
    // Subscribe to response
    gazebo::transport::SubscriberPtr responseSub =
      node->Subscribe("~/response", &OnResponse);

    // Publish request and wait for response
    boost::mutex::scoped_lock lock(g_mutex);
    requestPub->Publish(*g_requestMsg);
    g_condition.wait(lock);
  }

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr modelPub =
    node->Advertise<gazebo::msgs::Model>("~/model/modify");

  gazebo::msgs::Model modelMsg;
  modelMsg.set_name(name);
  modelMsg.set_id(g_modelMsg.id());
  gazebo::math::Pose pose(x, y, z, 0, 0, 0);
# if GAZEBO_MAJOR_VERSION >= 7
  gazebo::msgs::Set(modelMsg.mutable_pose(), pose.Ign());
# else
  gazebo::msgs::Set(modelMsg.mutable_pose(), pose);
# endif

  // Wait for a subscriber to connect
  modelPub->WaitForConnection();

  gazebo::common::Time::MSleep(100);
  modelPub->Publish(modelMsg);

  // Make sure to shut everything down.
  gazebo::transport::fini();
}

