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

/*
  // nominal
  ROS_INFO("harnessed.");
  std_msgs::String msg1;
  msg1.data = "harnessed";
  pubMode.publish(msg1);

  ros::Duration(0.3).sleep();

  // reset
  goal.params.behavior = atlas_msgs::AtlasSimInterface::RESET;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas reset action suceeded\n");

  ros::Duration(0.5).sleep();

  // safety
  goal.params.behavior = atlas_msgs::AtlasSimInterface::SAFETY;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas safety action suceeded\n");

  ros::Duration(0.5).sleep();

  // stand prep
  goal.params.behavior = atlas_msgs::AtlasSimInterface::STAND_PREP;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand prep action suceeded\n");

  ros::Duration(3).sleep();

  // nominal
  std_msgs::String msg;
  msg.data = "nominal";
  pubMode.publish(msg);

  ros::Duration(0.3).sleep();
*/

  // stand
  goal.params.behavior = atlas_msgs::AtlasSimInterface::STAND;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand action suceeded\n");

  ros::Duration(1).sleep();

  // demo1 - walk
  goal.params.behavior = atlas_msgs::AtlasSimInterface::MULTI_STEP_WALK;
  // goal.params.behavior = atlas_msgs::AtlasSimInterface::DEMO2;
  // goal.params.behavior = atlas_msgs::AtlasSimInterface::DEMO1;
  // build 10 steps
  unsigned int steps = 6;
  goal.params.multistep_walk_params.resize(steps+1);
  for (unsigned int i = 0; i < steps; ++i)
  {
    goal.params.multistep_walk_params[i].step_index = i + 1;
    goal.params.multistep_walk_params[i].foot_index = i % 2;
    goal.params.multistep_walk_params[i].duration = 0.63;
    goal.params.multistep_walk_params[i].pose.position.x = 0.2 * i;
    goal.params.multistep_walk_params[i].pose.position.y = 0.1 - (i%2)*0.2;
    goal.params.multistep_walk_params[i].pose.position.z = 0.1;
    goal.params.multistep_walk_params[i].pose.orientation.x = 0;
    goal.params.multistep_walk_params[i].pose.orientation.y = 0;
    goal.params.multistep_walk_params[i].pose.orientation.z = 0;
    goal.params.multistep_walk_params[i].pose.orientation.w = 1;
    goal.params.multistep_walk_params[i].swing_height = 0.3;
  }
  // add one more step, bring both feet together
  goal.params.multistep_walk_params[steps].step_index = steps + 1;
  goal.params.multistep_walk_params[steps].foot_index = steps % 2;
  goal.params.multistep_walk_params[steps].duration = 0.63;
  goal.params.multistep_walk_params[steps].pose.position.x = 0.2 * (steps - 1);
  goal.params.multistep_walk_params[steps].pose.position.y = 0.1-(steps%2)*0.2;
  goal.params.multistep_walk_params[steps].pose.position.z = 0.1;
  goal.params.multistep_walk_params[steps].pose.orientation.x = 0;
  goal.params.multistep_walk_params[steps].pose.orientation.y = 0;
  goal.params.multistep_walk_params[steps].pose.orientation.z = 0;
  goal.params.multistep_walk_params[steps].pose.orientation.w = 1;
  goal.params.multistep_walk_params[steps].swing_height = 0.3;

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas walk action suceeded\n");

  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
