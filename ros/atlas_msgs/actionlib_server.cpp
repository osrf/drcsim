// actionlib for BDI's dynamic controller
// see http://ros.org/wiki/actionlib for documentation on actions

#include "actionlib_server.h"

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
void ASIActionServer::ASIStateCB(
    const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(this->actionServerMutex);
  this->robotPosition.x = msg->pos_est.position.x;
  this->robotPosition.y = msg->pos_est.position.y;
  this->robotPosition.z = msg->pos_est.position.z;

  // 80 characters
  const atlas_msgs::AtlasBehaviorFeedback *fb =
    &(msg->behavior_feedback);

  typedef atlas_msgs::AtlasBehaviorFeedback ABFeedback;

  // Does the message contain bad news?
  // and should we do something about it other than letting the user
  // know we are in a bad state?
  switch (fb->status_flags)
  {
    case ABFeedback::STATUS_OK:
      ROS_DEBUG("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_OK]");
      break;
    case ABFeedback::STATUS_TRANSITION_IN_PROGRESS:
      ROS_DEBUG("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_TRANSITION_IN_PROGRESS]");
      break;
    case ABFeedback::STATUS_TRANSITION_SUCCESS:
      ROS_DEBUG("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_TRANSITION_SUCCESS]");
      break;
    case ABFeedback::STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_FAILED_TRANS_COM_POS:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_FAILED_TRANS_COM_POS]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_FAILED_TRANS_COM_VEL:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_FAILED_TRANS_COM_VEL]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_FAILED_TRANS_VEL:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_FAILED_TRANS_VEL]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_WARNING_AUTO_TRANS:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_WARNING_AUTO_TRANS]");
      this->executingGoal = false;
      return;
    case ABFeedback::STATUS_ERROR_FALLING:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "STATUS_ERROR_FALLING]");
      this->executingGoal = false;
      return;
    default:
      ROS_ERROR("AcitonServer: AtlasBehaviorFeedback error: ["
                "undocumented error state]");
      this->executingGoal = false;
      return;
  }

  // more check
  atlas_msgs::AtlasSimInterfaceCommand command;
  if (msg->desired_behavior != this->activeGoal.behavior)
  {
    // comment out until bdi controller functionality is complete
    // otherwise, we get occassional spam in some behaviors.
    //ROS_WARN("Switching behavior or setting behavior failed.");
  }

  // assuming there are no significant errors,
  // decide what to do based on whether a newGoal has been issued,
  // and if the robot is busy.
  if (!this->newGoal && !this->executingGoal)
  {
    // not doing anything right now, no new goal either,
    return;
  }
  else if (this->newGoal && this->executingGoal)
  {
    // robot's busy, but got new goal from user,
    // we need to preempt last goal, and start robot on new goal
    // put robot back to stand mode, and let controller take care of it.
    atlas_msgs::AtlasSimInterfaceCommand command;
    command.header = this->activeGoal.header;
    command.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
    this->atlasCommandPublisher.publish(command);
    this->executingGoal = false;
    return;
    // next time back in ASIStateCB, we'll be in newGoal&!executingGoal state.
    // Alternatively, give user an error and return.
  }
  else if (this->newGoal && !this->executingGoal)
  {
    // starting newGoal or executing one
    // simply set flags, and treat new goal as executing goal.
    this->newGoal = false;
    this->executingGoal = true;
    /// do startup stuff

    // copy goal info into command to be dispatched over
    // AtlasSimInterfaceCommand
    command.header = this->activeGoal.header;
    command.behavior = this->activeGoal.behavior;
    command.k_effort = this->activeGoal.k_effort;
    command.step_params = this->activeGoal.step_params;
    command.manipulate_params = this->activeGoal.manipulate_params;
    command.stand_params = this->activeGoal.stand_params;

    // do the proper thing based on behavior
    switch (this->activeGoal.behavior)
    {
      case atlas_msgs::WalkDemoGoal::WALK:
        {
          // ROS_ERROR("debug: csi[%d] nsin[%d] t_rem[%f] traj id[%d] size[%d]",
          //   (int)fb->walk_feedback.current_step_index,
          //   (int)fb->walk_feedback.next_step_index_needed,
          //   fb->walk_feedback.t_step_rem,
          //   (int)this->currentStepIndex,
          //   (int)this->activeGoal.steps.size());

          // if needed, publish next set of 4 commands
          for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
          {
            command.walk_params.step_data[i] =
              this->activeGoal.steps[this->currentStepIndex + i];
              std::cout << "  building stepId : " << i
                << "  traj id [" << this->currentStepIndex + i
                << "] step_index["
                << command.walk_params.step_data[i].step_index
                << "]  isRight["
                << command.walk_params.step_data[i].foot_index
                << "]  pos ["
                << command.walk_params.step_data[i].pose.position.x
                << ", "
                << command.walk_params.step_data[i].pose.position.y
                << "]\n";
          }
          // publish new set of commands
          this->atlasCommandPublisher.publish(command);
        }
        break;
      case atlas_msgs::WalkDemoGoal::STEP:
        {
          // fill in step command and pbulish it
        }
        break;
      case atlas_msgs::WalkDemoGoal::MANIPULATE:
        {
          // fill in manipulate command and pbulish it
        }
        break;
      case atlas_msgs::WalkDemoGoal::STAND_PREP:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::STAND:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::USER:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::FREEZE:
        // we don't need to do anything here
        break;
      default:
        break;
    }
  }
  else if (!this->newGoal && this->executingGoal)
  {
    // continue executing current goal

    // copy goal info into command to be dispatched over
    // AtlasSimInterfaceCommand
    command.header = this->activeGoal.header;
    command.behavior = this->activeGoal.behavior;
    command.k_effort = this->activeGoal.k_effort;
    command.step_params = this->activeGoal.step_params;
    command.manipulate_params = this->activeGoal.manipulate_params;
    command.stand_params = this->activeGoal.stand_params;

    // do the proper thing based on behavior
    switch (this->activeGoal.behavior)
    {
      case atlas_msgs::WalkDemoGoal::WALK:
        {
          int startIndex =
            std::min((long)fb->walk_feedback.next_step_index_needed,
              (long)this->activeGoal.steps.size() - NUM_REQUIRED_WALK_STEPS);

          // ROS_ERROR("debug: csi[%d] nsin[%d] t_rem[%f] traj id[%d] size[%d]",
          //   (int)fb->walk_feedback.current_step_index,
          //   (int)fb->walk_feedback.next_step_index_needed,
          //   fb->walk_feedback.t_step_rem,
          //   (int)this->currentStepIndex,
          //   (int)this->activeGoal.steps.size());

          //Is the sequence completed?
          if (fb->walk_feedback.current_step_index >=
                this->activeGoal.steps.size() - 1)
          {
            ROS_INFO("Walk trajectory completed, switching to stand mode.");
            command.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
            this->atlasCommandPublisher.publish(command);
            this->executingGoal = false;
            return;
          }

          // if needed, publish next set of 4 commands
          if (static_cast<int>(this->currentStepIndex) < startIndex)
          {
            this->currentStepIndex = static_cast<unsigned int>(startIndex);
            for (unsigned int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i)
            {
              command.walk_params.step_data[i] =
                this->activeGoal.steps[this->currentStepIndex + i];
                std::cout << "  building stepId : " << i
                  << "  traj id [" << this->currentStepIndex + i
                  << "] step_index["
                  << command.walk_params.step_data[i].step_index
                  << "]  isRight["
                  << command.walk_params.step_data[i].foot_index
                  << "]  pos ["
                  << command.walk_params.step_data[i].pose.position.x
                  << ", "
                  << command.walk_params.step_data[i].pose.position.y
                  << "]\n";
            }
            // publish new set of commands
            this->atlasCommandPublisher.publish(command);
          }
        }
        break;
      case atlas_msgs::WalkDemoGoal::STEP:
        {
          // fill in step command and pbulish it
        }
        break;
      case atlas_msgs::WalkDemoGoal::MANIPULATE:
        {
          // fill in manipulate command and pbulish it
        }
        break;
      case atlas_msgs::WalkDemoGoal::STAND_PREP:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::STAND:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::USER:
        // we don't need to do anything here
        break;
      case atlas_msgs::WalkDemoGoal::FREEZE:
        // we don't need to do anything here
        break;
      default:
        break;
    }
  }
  else
  {
    ROS_ERROR("ASIStateCB: Should never get here.");
  }
}

//TODO use this subscriber to get where the feet are located
////////////////////////////////////////////////////////////////////////////////
void ASIActionServer::atlasStateCB(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(this->actionServerMutex);
  tf::quaternionMsgToTF(msg->orientation, this->robotOrientation);
}

////////////////////////////////////////////////////////////////////////////////
void ASIActionServer::ActionServerCB()
{
  // actionlib simple action server
  // lock and set mode and params
  boost::mutex::scoped_lock lock(this->actionServerMutex);

  // When accepteNewGoal() is called, active goal (if any) is automatically
  // preempted.
  this->activeGoal = *this->actionServer->acceptNewGoal();
  if (this->activeGoal.behavior == atlas_msgs::WalkDemoGoal::WALK &&
          this->activeGoal.steps.size() < 2)
  {
      ROS_ERROR("Walk goal must contain two or more steps");
      this->executingGoal = false;
      this->newGoal = false;
      return;
  }

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
      agQuat = agQuat.normalize();
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
      this->currentStepIndex = 0;

      std::cout << "  building stepId : " << i
        << "  traj id [" << this->currentStepIndex + i
        << "] step_index["
        << this->activeGoal.steps[i].step_index
        << "]  isRight["
        << this->activeGoal.steps[i].foot_index
        << "]  pos ["
        << this->activeGoal.steps[i].pose.position.x
        << ", "
        << this->activeGoal.steps[i].pose.position.y
        << "]\n";

      ROS_INFO_STREAM("Step: " << i << " location- x: " <<
                      this->activeGoal.steps[i].pose.position.x <<
                      " y: " << this->activeGoal.steps[i].pose.position.y <<
                      " z: " << this->activeGoal.steps[i].pose.position.z);
  }

  for (unsigned int i = this->activeGoal.steps.size();
       i < NUM_REQUIRED_WALK_STEPS; ++i)
  {
      this->activeGoal.steps.push_back(this->activeGoal.steps[i-2]);
  }

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
