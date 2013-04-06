#include <std_msgs/String.h>
#include <atlas_msgs/AtlasSimInterfaceAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<atlas_msgs::AtlasSimInterfaceAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_client");
  Client client("atlas/bdi_control", true); // true -> don't need ros::spin()

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer();

  ros::NodeHandle rosNode("");
  ros::Publisher pubMode = rosNode.advertise<std_msgs::String>(
          "atlas/mode", 1, true);

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action

  atlas_msgs::AtlasSimInterfaceGoal goal;
  // Fill in goal here

  // nominal
  ros::Duration(5).sleep();
  std_msgs::String msg1;
  msg1.data = "harnessed";
  pubMode.publish(msg1);
  ros::Duration(0.3).sleep();

  // reset
  goal.params.behavior = 8;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas reset action suceeded\n");

  ros::Duration(5).sleep();

  // safety
  goal.params.behavior = 3;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas safety action suceeded\n");

  ros::Duration(5).sleep();

  // stand prep
  goal.params.behavior = 4;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand prep action suceeded\n");

  // nominal
  ros::Duration(5).sleep();
  std_msgs::String msg;
  msg.data = "nominal";
  pubMode.publish(msg);
  ros::Duration(0.3).sleep();

  // stand
  goal.params.behavior = 7;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand action suceeded\n");

  ros::Duration(5).sleep();

  // demo1 - walk
  goal.params.behavior = 9;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas demo1 action suceeded\n");

  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
