/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <ros/ros.h>
#include <AtlasInterface.h>
#include <AtlasInterfaceTypes.h>
#include <AtlasUtility.h>

#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>

// This is not the cleanest thing to do, but it does avoid code duplication in
// the command message callback.
#define ROS_MSG_VECTOR_SIZE_CHECK(v) \
      if ((v).size() != Atlas::NUM_JOINTS)\
      {\
        ROS_ERROR("AtlasCommand message contains a different number (%ld) of "\
                  "elements in " #v " than expected (%d)",\
                  (v).size(),\
                  Atlas::NUM_JOINTS);\
        this->last_atlas_cmd_msg_valid = false;\
        return true;\
      }

// Uncomment and recompile to do basic testing without a robot available
//#define NOROBOT

class AtlasInterfaceBridge
{
  private:
    AtlasInterface the_atlas;
    AtlasUtility u_atlas;
    ros::Publisher atlas_state_pub;
    ros::Subscriber atlas_cmd_sub;
    ros::NodeHandle node;
    boost::mutex msg_lock;
    atlas_msgs::AtlasCommand last_atlas_cmd_msg;
    atlas_msgs::AtlasState last_atlas_state_msg;
    bool last_atlas_cmd_msg_valid;

    void atlasCommandCallback(const atlas_msgs::AtlasCommandConstPtr& msg);

  public:
    AtlasInterfaceBridge() : last_atlas_cmd_msg_valid(false) {}
    ~AtlasInterfaceBridge() {}
    // Initialize the connection to the robot.  Must be called first.
    bool start(int argc, char** argv);
    // Shut down the connection to the robot.
    bool stop();
    // Do one iteration of the interaction loop with the robot.
    // Returns:
    //  true if the update was successful or at least survivable (keep going)
    //  false if a significant error occurred (time to stop)
    bool update();
};

bool AtlasInterfaceBridge::start(int argc, char** argv)
{
  ROS_INFO("Opening connection to robot.");
  bool result;
#if ! defined (NOROBOT)
  result = this->u_atlas.start_control_session(argc, argv);
#else
  result = true;
#endif

  this->atlas_cmd_sub = this->node.subscribe("atlas/atlas_command", 1,
    &AtlasInterfaceBridge::atlasCommandCallback, this);
  this->atlas_state_pub = 
    this->node.advertise<atlas_msgs::AtlasState>("atlas/atlas_state", 1);
  this->last_atlas_state_msg.position.resize(Atlas::NUM_JOINTS);
  this->last_atlas_state_msg.velocity.resize(Atlas::NUM_JOINTS);
  this->last_atlas_state_msg.effort.resize(Atlas::NUM_JOINTS);
  // TODO: also allocate space for the gains
  
  //
  // Set position control joint gains.
  //
  this->u_atlas.set_position_control_gains();

  return result;
}

bool AtlasInterfaceBridge::stop()
{
  ROS_INFO("Closing connection to robot.");
  if (this->atlas_cmd_sub)
    this->atlas_cmd_sub.shutdown();
  if (this->atlas_state_pub)
    this->atlas_state_pub.shutdown();
  bool result;
#if ! defined (NOROBOT)
  result = this->u_atlas.stop_control_session();
#else
  result = true;
#endif
  return result;
}

bool
AtlasInterfaceBridge::update()
{
  //
  // Read any available data from the robot.  Use the AtlasUtility function
  //  recv_data_from_robot().  It will wait (up to the specified time)
  //  for data from the robot, and then read all pending data.
  //
  // wait up to 1 second for data
#if ! defined (NOROBOT)
  double timeout = 1.0;
  bool data_received = this->u_atlas.recv_data_from_robot(timeout);

  if (!data_received)
  {
    ROS_WARN("Did not receive data from robot within %f second(s)", timeout);
    // TODO: return false here?  I don't know how serious this condition is.
    return true;
  }
#endif

  // Fill out and publish the received data
  // TODO: if this step is too slow, then try_lock and copy the data for
  // publication from another thread.
  for (unsigned int i = 0; i < Atlas::NUM_JOINTS; ++i)
  {
    this->last_atlas_state_msg.position[i] = s_data_from_robot.j[i].q;
    this->last_atlas_state_msg.velocity[i] = s_data_from_robot.j[i].qd;
    this->last_atlas_state_msg.effort[i] = s_data_from_robot.j[i].f;
    // TODO: also fill in other arrays
  }
  
  // Atlas gives time in microseconds since the epoch.
  this->last_atlas_state_msg.header.stamp.fromSec(
    s_data_from_robot.timestamp / 1e6);
  this->atlas_state_pub.publish(this->last_atlas_state_msg);

  //
  // Check for faults.  Don't exit if we hit one; wait for stop state.
  //
  this->u_atlas.check_robot_faults();

  //
  // Check for unexpected change to stop.
  //
  if (s_data_from_robot.run_state == RUN_STATE_STOP)
  {
    ROS_WARN("Robot control unexpectedly interrupted by stop state.");

    this->u_atlas.handle_robot_stop();
    return false;
  }

  // Try to fill out the robot command packet from the latest ROS command
  // message.
  bool data_to_send = false;
  {
    // Try to lock on the shared copy of the incoming command, but don't wait
    // for it.
    boost::mutex::scoped_try_lock lock(this->msg_lock);
    if (lock.owns_lock() && this->last_atlas_cmd_msg_valid)
    {
      // Sanity checks; the check macro will return out of this function if
      // there's a mismatch in size.
      ROS_MSG_VECTOR_SIZE_CHECK(this->last_atlas_cmd_msg.position);
      ROS_MSG_VECTOR_SIZE_CHECK(this->last_atlas_cmd_msg.velocity);
      ROS_MSG_VECTOR_SIZE_CHECK(this->last_atlas_cmd_msg.effort);
      // TODO: also check the arrays of gains

      // Here I'm assuming that the ordering of joints dictated by the ROS
      // message atlas_msgs/AtlasState agrees with the ordering of joints
      // assumed by the Atlas API.
      for (unsigned int i = 0; i < Atlas::NUM_JOINTS; ++i)
      {
        s_data_to_robot.j[i].q_d = this->last_atlas_cmd_msg.position[i];
        s_data_to_robot.j[i].qd_d = this->last_atlas_cmd_msg.velocity[i];
        s_data_to_robot.j[i].f_d = this->last_atlas_cmd_msg.effort[i];

        // TODO: also apply the gains in the message
      }

      // Atlas wants time in milliseconds since the epoch.
      s_data_to_robot.timestamp = 
        this->last_atlas_cmd_msg.header.stamp.toSec() * 1e3;

      data_to_send = true;
    }

  }

  //
  // If we succeeded in filling out the packet, send control data to robot.
  //
  if (data_to_send)
  {
    ROS_INFO("Sending control data to robot.");
#if ! defined (NOROBOT)
    AtlasErrorCode ec;
    ec = this->the_atlas.send_control_data_to_robot(s_data_to_robot,
      &s_sent_control_seq_id);
    this->u_atlas.check_error_code(ec, "send_control_data_to_robot()");
#endif
  }

  return true;
}

void 
AtlasInterfaceBridge::atlasCommandCallback(
  const atlas_msgs::AtlasCommandConstPtr& msg)
{
  // Lock and copy the incoming command to where the main loop will pick it up
  boost::mutex::scoped_lock lock(this->msg_lock);
  ROS_INFO("Received a message");
  this->last_atlas_cmd_msg = *msg;
  this->last_atlas_cmd_msg_valid = true;
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_interface_bridge");

  AtlasInterfaceBridge aib;

  if (!aib.start(argc, argv))
  {
    ROS_ERROR("Failed to start Atlas robot.");
    exit(1);
  }

  // Service message callbacks in another thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Entering update loop");
  while (ros::ok() && aib.update());
  ROS_INFO("Exited update loop");

  spinner.stop();

  if (!aib.stop())
  {
    ROS_ERROR("Failed to stop Atlas robot.");
    exit(1);
  }

  return 0;
}


