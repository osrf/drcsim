// actionlib for BDI's dynamic controller
// see http://ros.org/wiki/actionlib for documentation on actions

#include "actionlib_server.h"

//////////////////////////////////////////////////
ASIActionServer::ASIActionServer()
{
  this->actionServer = new ActionServer(this->rosNode, "atlas/bdi_control",
                                        false);
  // Register goal callback
  this->actionServer->registerGoalCallback(
    boost::bind(&ASIActionServer::ActionServerCB, this));


    this->ASIStateSubscriber =
      this->rosNode.subscribe("atlas/atlas_sim_interface_state", 10,
        &ASIActionServer::ASIStateCB, this);

    this->atlasStateSubscriber =
      this->rosNode.subscribe("atlas/atlas_state", 10,
        &ASIActionServer::atlasStateCB, this);

  this->atlasCommandPublisher =
    this->rosNode.advertise<atlas_msgs::AtlasSimInterfaceCommand>(
      "atlas/atlas_sim_interface_command", 1);

  this->actionServer->start();
  ros::spin();
}

//////////////////////////////////////////////////
void ASIActionServer::ASIStateCB(
    const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(this->actionServerMutex);
  this->robotPosition.x = msg->pos_est.position.x;
  this->robotPosition.y = msg->pos_est.position.y;
  this->robotPosition.z = msg->pos_est.position.z;

  // Is there a goal to execute?
  if (!this->executingGoal)
  {
    atlas_msgs::AtlasSimInterfaceCommand command;
    command.header = this->activeGoal.header;
    command.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
    this->atlasCommandPublisher.publish(command);
    return;
  }

  // Does the message contain bad news?
  if (msg->behavior_feedback.status_flags > 2)
  {
    ROS_INFO("Canceling goal, received error");
    this->executingGoal = false;
    return;
  }

  atlas_msgs::AtlasSimInterfaceCommand command;
  if (msg->desired_behavior != this->activeGoal.behavior)
  {
    //ROS_INFO("Switching behavior");
  }

  command.header = this->activeGoal.header;
  command.behavior = this->activeGoal.behavior;
  command.k_effort = this->activeGoal.k_effort;
  command.step_params = this->activeGoal.step_params;
  command.manipulate_params = this->activeGoal.manipulate_params;
  command.stand_params = this->activeGoal.stand_params;
  if (this->activeGoal.behavior == atlas_msgs::WalkDemoGoal::WALK)
  {
    //Is the sequence completed?
    if (!this->newGoal &&
        msg->behavior_feedback.walk_feedback.next_step_index_needed >=
          this->activeGoal.steps.size())
    {
      ROS_INFO("Walk trajectory completed, standing");
      command.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
      this->atlasCommandPublisher.publish(command);
      this->executingGoal = false;
      return;
    }

    unsigned int start_index =
      std::min((long)msg->behavior_feedback.walk_feedback.next_step_index_needed,
        (long)this->activeGoal.steps.size() - NUM_REQUIRED_WALK_STEPS);
    for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
    {
      command.walk_params.step_data[i] =
        this->activeGoal.steps[start_index + i];
    }
  }
  //ROS_INFO("Publishing command");
  this->atlasCommandPublisher.publish(command);

  this->newGoal = (this->newGoal &&
    msg->behavior_feedback.walk_feedback.next_step_index_needed >=
       NUM_REQUIRED_WALK_STEPS);
}

//TODO use this subscriber to get where the feet are located
//////////////////////////////////////////////////
void ASIActionServer::atlasStateCB(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  tf::quaternionMsgToTF(msg->orientation, this->robotOrientation);
}

//////////////////////////////////////////////////
void ASIActionServer::ActionServerCB()
{
  // actionlib simple action server
  // lock and set mode and params
  boost::mutex::scoped_lock lock(this->actionServerMutex);

  // When accepteNewGoal() is called, active goal (if any) is automatically
  // preempted.
  this->activeGoal = *this->actionServer->acceptNewGoal();

  ROS_INFO_STREAM("Current position - x: " << this->robotPosition.x <<
                  " y: " << this->robotPosition.y <<
                  " z: " << this->robotPosition.z);
  for (unsigned int i = 0; i < this->activeGoal.steps.size(); ++i)
  {
      // Position vector of the robot
      tf::Vector3 rOPos = tf::Vector3(this->robotPosition.x,
                                      this->robotPosition.y,
                                      this->robotPosition.z);

      // Create transform of this active goal step
      tf::Quaternion agQuat;
      tf::quaternionMsgToTF(this->activeGoal.steps[i].pose.orientation, agQuat);
      tf::Transform aGTransform(agQuat,
        tf::Vector3(this->activeGoal.steps[i].pose.position.x,
                    this->activeGoal.steps[i].pose.position.y,
                    this->activeGoal.steps[i].pose.position.z));

      // We only want to transform with respect to the robot's yaw
      double yaw = tf::getYaw(this->robotOrientation);

      // Transform of the robot in world coordinates
      tf::Transform transform(tf::createQuaternionFromYaw(yaw), rOPos);

      // Transform the active goal step to world pose.
      tf::Transform newTransform = transform * aGTransform;

      ROS_INFO_STREAM("Before xform. Step: " << i << " location- x: " <<
                      this->activeGoal.steps[i].pose.position.x <<
                      " y: " << this->activeGoal.steps[i].pose.position.y <<
                      " z: " << this->activeGoal.steps[i].pose.position.z);


      // Create geometry_msgs transform msg and change the active goal to
      // reflect the transform
      geometry_msgs::Transform transformMsg;
      tf::transformTFToMsg(newTransform, transformMsg);
      this->activeGoal.steps[i].pose.orientation = transformMsg.rotation;
      this->activeGoal.steps[i].pose.position.x = transformMsg.translation.x;
      this->activeGoal.steps[i].pose.position.y = transformMsg.translation.y;
      this->activeGoal.steps[i].pose.position.z = transformMsg.translation.z;

      ROS_INFO_STREAM("Step: " << i << " location- x: " <<
                      this->activeGoal.steps[i].pose.position.x <<
                      " y: " << this->activeGoal.steps[i].pose.position.y <<
                      " z: " << this->activeGoal.steps[i].pose.position.z);
  }
  atlas_msgs::AtlasSimInterfaceCommand command;
  command.header = this->activeGoal.header;
  command.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
  this->atlasCommandPublisher.publish(command);

  ROS_INFO("Received goal, executing");
  this->executingGoal = true;
  this->newGoal = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atlas_bdi_control");
  ASIActionServer();

  // actionlib simple action server

  return 0;
}
