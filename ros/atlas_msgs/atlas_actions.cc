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

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action

  atlas_msgs::AtlasSimInterfaceGoal goal;
  // Fill in goal here

  // stand prep
  goal.params.behavior = 4;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand prep action suceeded");

  ros::Duration(5).sleep();

  // stand
  goal.params.behavior = 7;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand action suceeded");

  ros::Duration(1).sleep();

  // demo1 - walk
  goal.params.behavior = 9;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(1.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas demo1 action suceeded");

  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
