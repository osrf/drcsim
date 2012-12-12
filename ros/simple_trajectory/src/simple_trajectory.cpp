#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/drc_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    ROS_INFO("starting trajectory");
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

    ROS_INFO("sending trajectory");
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with four waypoints, used as an example
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("l_leg_uhz");
    goal.trajectory.joint_names.push_back("l_leg_mhx");
    goal.trajectory.joint_names.push_back("l_leg_lhy");
    goal.trajectory.joint_names.push_back("l_leg_kny");
    goal.trajectory.joint_names.push_back("l_leg_uay");
    goal.trajectory.joint_names.push_back("l_leg_lax");

    goal.trajectory.joint_names.push_back("r_leg_uhz");
    goal.trajectory.joint_names.push_back("r_leg_mhx");
    goal.trajectory.joint_names.push_back("r_leg_lhy");
    goal.trajectory.joint_names.push_back("r_leg_kny");
    goal.trajectory.joint_names.push_back("r_leg_uay");
    goal.trajectory.joint_names.push_back("r_leg_lax");

    goal.trajectory.joint_names.push_back("l_arm_usy");
    goal.trajectory.joint_names.push_back("l_arm_shx");
    goal.trajectory.joint_names.push_back("l_arm_ely");
    goal.trajectory.joint_names.push_back("l_arm_elx");
    goal.trajectory.joint_names.push_back("l_arm_uwy");
    goal.trajectory.joint_names.push_back("l_arm_mwx");

    goal.trajectory.joint_names.push_back("r_arm_usy");
    goal.trajectory.joint_names.push_back("r_arm_shx");
    goal.trajectory.joint_names.push_back("r_arm_ely");
    goal.trajectory.joint_names.push_back("r_arm_elx");
    goal.trajectory.joint_names.push_back("r_arm_uwy");
    goal.trajectory.joint_names.push_back("r_arm_mwx");

    goal.trajectory.joint_names.push_back("neck_ay"  );
    goal.trajectory.joint_names.push_back("back_lbz" );
    goal.trajectory.joint_names.push_back("back_mby" );
    goal.trajectory.joint_names.push_back("back_ubx" );

    // We will have four waypoints in this goal trajectory
    goal.trajectory.points.resize(4);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(28);
    goal.trajectory.points[ind].positions[0]  = 0.0;
    goal.trajectory.points[ind].positions[1]  = -0.1;
    goal.trajectory.points[ind].positions[2]  = 0.0;
    goal.trajectory.points[ind].positions[3]  = 0.0;
    goal.trajectory.points[ind].positions[4]  = 0.1;
    goal.trajectory.points[ind].positions[5]  = 0.3;

    goal.trajectory.points[ind].positions[6]  = 0.0;
    goal.trajectory.points[ind].positions[7]  = -0.7;
    goal.trajectory.points[ind].positions[8]  = -1.2;
    goal.trajectory.points[ind].positions[9]  = 1.4;
    goal.trajectory.points[ind].positions[10] = -1.4;
    goal.trajectory.points[ind].positions[11] = 0.0;

    goal.trajectory.points[ind].positions[12] = 0.0;
    goal.trajectory.points[ind].positions[13] = 0.0;
    goal.trajectory.points[ind].positions[14] = 0.0;
    goal.trajectory.points[ind].positions[15] = 0.0;
    goal.trajectory.points[ind].positions[16] = 0.0;
    goal.trajectory.points[ind].positions[17] = 0.0;

    goal.trajectory.points[ind].positions[18] = 0.0;
    goal.trajectory.points[ind].positions[19] = 0.0;
    goal.trajectory.points[ind].positions[20] = 0.0;
    goal.trajectory.points[ind].positions[21] = 0.0;
    goal.trajectory.points[ind].positions[22] = 0.0;
    goal.trajectory.points[ind].positions[23] = 0.0;

    goal.trajectory.points[ind].positions[24] = 0.0;
    goal.trajectory.points[ind].positions[25] = 0.0;
    goal.trajectory.points[ind].positions[26] = 0.0;
    goal.trajectory.points[ind].positions[27] = 0.2;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(28);
    for (size_t j = 0; j < 28; ++j)
    {
      // Zero velocity lets the controller pick
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after the start of the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(28);
    goal.trajectory.points[ind].positions[0]  = 0.0;
    goal.trajectory.points[ind].positions[1]  = 0.4;
    goal.trajectory.points[ind].positions[2]  =  1.2;
    goal.trajectory.points[ind].positions[3]  =  0.8;
    goal.trajectory.points[ind].positions[4]  = -1.4;
    goal.trajectory.points[ind].positions[5]  = 0.2;

    goal.trajectory.points[ind].positions[6]  = 0.0;
    goal.trajectory.points[ind].positions[7]  = -0.7;
    goal.trajectory.points[ind].positions[8]  = -1.2;
    goal.trajectory.points[ind].positions[9]  = 1.4;
    goal.trajectory.points[ind].positions[10] = -1.4;
    goal.trajectory.points[ind].positions[11] = 0.0;

    goal.trajectory.points[ind].positions[12] = 0.0;
    goal.trajectory.points[ind].positions[13] = 0.0;
    goal.trajectory.points[ind].positions[14] = 0.0;
    goal.trajectory.points[ind].positions[15] = 0.0;
    goal.trajectory.points[ind].positions[16] = 0.0;
    goal.trajectory.points[ind].positions[17] = 0.0;

    goal.trajectory.points[ind].positions[18] = 0.0;
    goal.trajectory.points[ind].positions[19] = 0.0;
    goal.trajectory.points[ind].positions[20] = 0.0;
    goal.trajectory.points[ind].positions[21] = 0.0;
    goal.trajectory.points[ind].positions[22] = 0.0;
    goal.trajectory.points[ind].positions[23] = 0.0;

    goal.trajectory.points[ind].positions[24] = 0.0;
    goal.trajectory.points[ind].positions[25] = 0.0;
    goal.trajectory.points[ind].positions[26] = 0.0;
    goal.trajectory.points[ind].positions[27] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(28);
    for (size_t j = 0; j < 28; ++j)
    {
      // Zero velocity lets the controller pick
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1.5 seconds after the start of the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.5);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(28);
    goal.trajectory.points[ind].positions[0]  = 0;
    goal.trajectory.points[ind].positions[1]  = 0;
    goal.trajectory.points[ind].positions[2]  = 0;
    goal.trajectory.points[ind].positions[3]  = 0;
    goal.trajectory.points[ind].positions[4]  = 0.1;
    goal.trajectory.points[ind].positions[5]  = 0;

    goal.trajectory.points[ind].positions[6]  = 0.0;
    goal.trajectory.points[ind].positions[7]  = 0.0;
    goal.trajectory.points[ind].positions[8]  = -0.2;
    goal.trajectory.points[ind].positions[9]  =  0.4;
    goal.trajectory.points[ind].positions[10] = -0.4;
    goal.trajectory.points[ind].positions[11] = 0.0;

    goal.trajectory.points[ind].positions[12] = 0.0;
    goal.trajectory.points[ind].positions[13] = 0.0;
    goal.trajectory.points[ind].positions[14] = 0.0;
    goal.trajectory.points[ind].positions[15] = 0.0;
    goal.trajectory.points[ind].positions[16] = 0.0;
    goal.trajectory.points[ind].positions[17] = 0.0;

    goal.trajectory.points[ind].positions[18] = 0.0;
    goal.trajectory.points[ind].positions[19] = 0.0;
    goal.trajectory.points[ind].positions[20] = 0.0;
    goal.trajectory.points[ind].positions[21] = 0.0;
    goal.trajectory.points[ind].positions[22] = 0.0;
    goal.trajectory.points[ind].positions[23] = 0.0;

    goal.trajectory.points[ind].positions[24] = 0.0;
    goal.trajectory.points[ind].positions[25] = 0.0;
    goal.trajectory.points[ind].positions[26] = 1.0;
    goal.trajectory.points[ind].positions[27] = 1.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(28);
    for (size_t j = 0; j < 28; ++j)
    {
      // Zero velocity lets the controller pick
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 3 seconds after the start of the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    // Third trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(28);
    goal.trajectory.points[ind].positions[0]  = 0.0;
    goal.trajectory.points[ind].positions[1]  = 0.0;
    goal.trajectory.points[ind].positions[2]  = 0.0;
    goal.trajectory.points[ind].positions[3]  = 0.0;
    goal.trajectory.points[ind].positions[4]  = 0.0;
    goal.trajectory.points[ind].positions[5]  = 0.0;

    goal.trajectory.points[ind].positions[6]  = 0.0;
    goal.trajectory.points[ind].positions[7]  = 0.0;
    goal.trajectory.points[ind].positions[8]  = 0.0;
    goal.trajectory.points[ind].positions[9]  = 0.0;
    goal.trajectory.points[ind].positions[10] = 0.0;
    goal.trajectory.points[ind].positions[11] = 0.0;

    goal.trajectory.points[ind].positions[12] = 0.0;
    goal.trajectory.points[ind].positions[13] = 0.0;
    goal.trajectory.points[ind].positions[14] = 0.0;
    goal.trajectory.points[ind].positions[15] = 0.0;
    goal.trajectory.points[ind].positions[16] = 0.0;
    goal.trajectory.points[ind].positions[17] = 0.0;

    goal.trajectory.points[ind].positions[18] = 0.0;
    goal.trajectory.points[ind].positions[19] = 0.0;
    goal.trajectory.points[ind].positions[20] = 0.0;
    goal.trajectory.points[ind].positions[21] = 0.0;
    goal.trajectory.points[ind].positions[22] = 0.0;
    goal.trajectory.points[ind].positions[23] = 0.0;

    goal.trajectory.points[ind].positions[24] = 0.0;
    goal.trajectory.points[ind].positions[25] = 0.0;
    goal.trajectory.points[ind].positions[26] = 0.0;
    goal.trajectory.points[ind].positions[27] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(28);
    for (size_t j = 0; j < 28; ++j)
    {
      // Zero velocity lets the controller pick
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 4 seconds after the start of the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    // tolerances
    //for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
    //{
      //control_msgs::JointTolerance jt;
      //jt.name = goal.trajectory.joint_names[j];
      //jt.position = 1000;
      //jt.velocity = 1000;
      //jt.acceleration = 1000;
      //goal.goal_tolerance.push_back(jt);
    //}

    //goal.goal_time_tolerance.sec = 10;
    //goal.goal_time_tolerance.nsec = 0;

    // we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  ros::NodeHandle rh;

  // On startup, wait until we've started receiving the time feed from
  // simulation.  Otherwise we might send improperly timestamped trajectories.
  while (ros::Time::now().toSec() == 0)
  {
    // Use WallDuration because we don't have a time feed yet.
    ros::WallDuration(0.1).sleep();
  }

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
}
