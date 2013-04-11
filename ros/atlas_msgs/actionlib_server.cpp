// actionlib for BDI's dynamic controller
// see http://ros.org/wiki/actionlib for documentation on actions
#include <atlas_msgs/AtlasSimInterfaceAction.h>
#include <actionlib/server/simple_action_server.h>


  // actionlib simple action server
  typedef actionlib::SimpleActionServer<atlas_msgs::AtlasSimInterfaceAction>
    ActionServer;

    /// \brief actionlib simple action server executor callback
    private: void ActionServerCallback();

    /// \brief lock while updating control modes
    private: boost::mutex actionServerMutex;

    /// \brief actionlib simple action server
    private: ActionServer* actionServer;

    /// \brief local copy of the goal
    private: atlas_msgs::AtlasSimInterfaceGoal activeGoal;

    /// \brief actionlib feedback
    private: atlas_msgs::AtlasSimInterfaceFeedback actionServerFeedback;

    /// \brief actionlib result
    private: atlas_msgs::AtlasSimInterfaceResult actionServerResult;

    /// \brief used for trajectory rollout
    private: std::vector<atlas_msgs::AtlasBehaviorStepData>
      stepTrajectory;

    /// \brief current position of the robot
    math::Vector3 currentPelvisPosition;
    math::Vector3 currentLFootPosition;
    math::Vector3 currentRFootPosition;
    math::Pose bdiOdometryFrame;

    /// \brief fill in action server feedback state from toRobot,
    /// where toRobot is populated by call to AtlasSimInterface
    /// process_control_input()
    private: void UpdateActionServerStateFeedback();


    // walking parameters
    private: double strideSagittal;
    private: double strideCoronal;
    private: double stepWidth;
    private: double strideDuration;
    private: double walkYawRate;



////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::UpdateActionServerStateFeedback()
{
  // populate AtlasSimInterfaceState.msg and publish it
  this->actionServerFeedback.state.error_code =
    this->actionServerResult.end_state.error_code;

  // switch to fb?
  // this->actionServerFeedback.state.current_step_index =
  //   this->atlasControlOutput.current_step_index;

  this->actionServerFeedback.state.pelvis_position.x =
    this->atlasControlOutput.pos_est.position.n[0];
  this->actionServerFeedback.state.pelvis_position.y =
    this->atlasControlOutput.pos_est.position.n[1];
  this->actionServerFeedback.state.pelvis_position.z =
    this->atlasControlOutput.pos_est.position.n[2];

  this->actionServerFeedback.state.pelvis_velocity.x =
    this->atlasControlOutput.pos_est.velocity.n[0];
  this->actionServerFeedback.state.pelvis_velocity.y =
    this->atlasControlOutput.pos_est.velocity.n[1];
  this->actionServerFeedback.state.pelvis_velocity.z =
    this->atlasControlOutput.pos_est.velocity.n[1];

  /// \TODO: this gets the positions, but not orientations
  this->actionServerFeedback.state.foot_pose_est[0].position.x =
    this->atlasControlOutput.foot_pos_est[0].n[0];
  this->actionServerFeedback.state.foot_pose_est[0].position.y =
    this->atlasControlOutput.foot_pos_est[0].n[1];
  this->actionServerFeedback.state.foot_pose_est[0].position.z =
    this->atlasControlOutput.foot_pos_est[0].n[2];

  /// \TODO: this gets the positions, but not orientations
  this->actionServerFeedback.state.foot_pose_est[1].position.x =
    this->atlasControlOutput.foot_pos_est[1].n[0];
  this->actionServerFeedback.state.foot_pose_est[1].position.y =
    this->atlasControlOutput.foot_pos_est[1].n[1];
  this->actionServerFeedback.state.foot_pose_est[1].position.z =
    this->atlasControlOutput.foot_pos_est[1].n[2];

  if (this->actionServer->isActive())
    this->actionServer->publishFeedback(this->actionServerFeedback);
}

  // demo1
  ros::SubscribeOptions bdiCmdVelSo =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    "atlas/bdi_cmd_vel", 100,
    boost::bind(&AtlasPlugin::SetBDICmdVel, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subBDICmdVel = this->rosNode->subscribe(bdiCmdVelSo);

    /// \brief demo1
    public: void SetBDICmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);
    public: ros::Subscriber subBDICmdVel;
    public: math::Vector3 demo1Vel;

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetBDICmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  // update demo1 velocity command
  this->demo1Vel.x = _cmd->linear.x * this->strideSagittal;
  this->demo1Vel.y = _cmd->linear.y * this->strideCoronal;
  this->demo1Vel.z = _cmd->angular.z * this->walkYawRate;
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::ActionServerCallback()
{
  // actionlib simple action server
  // lock and set mode and params
  boost::mutex::scoped_lock lock(this->actionServerMutex);

  // When accepteNewGoal() is called, active goal (if any) is automatically
  // preempted.
  this->activeGoal = *this->actionServer->acceptNewGoal();

      std::string mode;
      this->actionServerResult.end_state.error_code =
        this->atlasSimInterface->get_desired_behavior(mode);

      if (this->actionServerResult.end_state.error_code != NO_ERRORS)
      {
        ROS_ERROR("AtlasSimInterface: get_desired_behavior failed with "
                  "error code (%d), controller update skipped.",
                  this->actionServerResult.end_state.error_code);
        this->actionServerResult.success = false;
        this->actionServer->setAborted(this->actionServerResult);
      }

  switch (this->activeGoal.params.behavior)
  {
    case atlas_msgs::AtlasSimInterface::USER:
      {
        // is this an effective way to switch back to PID control
        // from bdi controller that uses feedforward torques?
        ROS_INFO("AtlasSimInterface: set control mode to User.");
        // revert to PID control
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("User");
        // clear out forces
        for (unsigned i = 0; i < this->jointNames.size(); ++i)
          this->atlasControlOutput.f_out[i] = 0;
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          this->actionServerResult.success = true;
          this->actionServer->setSucceeded(this->actionServerResult);
        }
        else
        {
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::PID_OFF:
      {
        ROS_INFO("Set PID gains to zero.");
        this->ZeroAtlasCommand();
        this->actionServerResult.success = true;
        this->actionServer->setSucceeded(this->actionServerResult);
      }
      break;
    case atlas_msgs::AtlasSimInterface::RELOAD_PID:
      {
        ROS_INFO("Reload PID gains from param server.");
        this->LoadPIDGainsFromParameter();
        this->actionServerResult.success = true;
        this->actionServer->setSucceeded(this->actionServerResult);
      }
      break;
    case atlas_msgs::AtlasSimInterface::FREEZE:
      {
        ROS_INFO("AtlasSimInterface: Freeze mode.");
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Freeze");
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          this->actionServerResult.success = true;
          this->actionServer->setSucceeded(this->actionServerResult);
        }
        else
        {
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::STAND_PREP:
      {
        ROS_INFO("AtlasSimInterface: StandPrep mode.");
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("StandPrep");
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          this->actionServerResult.success = true;
          this->actionServer->setSucceeded(this->actionServerResult);
        }
        else
        {
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::DEMO1:
      {
        // Set DEMO1 Goal
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Walk");

        // get current location of the robot in its internal
        // odometry frame
        {
          // Flag for getting state on the first time
          // this->resetAtlasRobotState = true;

          // call process_control_input once to get atlasControlOutput
          // with internal state, so we know where we are in
          // the robot's internal odometry.
          // this->actionServerResult.end_state.error_code =
          //   this->atlasSimInterface->process_control_input(
          //   this->atlasControlInput, this->atlasRobotState,
          //   this->atlasControlOutput);

          // get current foot placement
          this->currentPelvisPosition =
            this->ToVec3(this->atlasControlOutput.pos_est.position);
          this->currentLFootPosition =
            this->ToVec3(this->atlasControlOutput.foot_pos_est[0]);
          this->currentRFootPosition =
            this->ToVec3(this->atlasControlOutput.foot_pos_est[1]);

          // deduce current orientation from feet location relative
          // to pelvis.
          math::Vector3 dr = currentRFootPosition - currentLFootPosition;
          double currentOrientation = atan2(dr.x, -dr.y);

          // try to create current odometry pose, but ideally we would like
          // this from the controller.
          this->bdiOdometryFrame = math::Pose(this->currentPelvisPosition,
            math::Quaternion(0, 0, currentOrientation));

          // gzdbg <<   "current position pelvis["
          //       << this->currentPelvisPosition
          //       << "] current position l_foot["
          //       << this->currentLFootPosition
          //       << "] current position r_root["
          //       << this->currentRFootPosition
          //       << "] orientation [" << currentOrientation
          //       << "]\n";
        }

        // build initial step using current local coordinate frame
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Walk");

        AtlasBehaviorWalkFeedback *fb =
          &this->atlasControlOutput.behavior_feedback.walk_feedback;
        // gzerr << " err [" << this->actionServerResult.end_state.error_code
        //       << "] fbcsi[" << fb->current_step_index
        //       << "] fbnsi[" << fb->next_step_index_needed
        //       << "] trem[" << fb->t_step_rem
        //       << "\n";

        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: starting demo1 mode.");
          // set goal's use_demo_walk to false
          AtlasBehaviorWalkParams* walkParams =
            &this->atlasControlInput.walk_params;
          walkParams->use_demo_walk = false;

          // get last foot index
          bool footIndex = walkParams->step_data[0].foot_index;

          // initialize buffer with first 4 steps
          for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS;
               ++stepId)
          {
            walkParams->step_data[stepId].step_index =
              fb->current_step_index + stepId;
            // alternate foot
            footIndex = !footIndex;
            walkParams->step_data[stepId].foot_index = footIndex;
            walkParams->step_data[stepId].duration = this->strideDuration;
            double coronalOffset = (1.0 - 2.0*footIndex)*this->stepWidth;
            walkParams->step_data[stepId].position =
              this->ToVec3(this->bdiOdometryFrame.pos +
                           math::Vector3(0, coronalOffset, 0));
            // set initial yaw rate to 0
            walkParams->step_data[stepId].yaw = 0;

            // gzdbg <<   "Building stepId [" << stepId
            //       << "] step_index["
            //       << walkParams->step_data[stepId].step_index
            //       << "]  isRight["
            //       << walkParams->step_data[stepId].foot_index
            //       << "]  pos ["
            //       << walkParams->step_data[stepId].position.n[0]
            //       << ", "
            //       << walkParams->step_data[stepId].position.n[1]
            //       << ", "
            //       << walkParams->step_data[stepId].position.n[2]
            //       << "]\n";
          }
        }
        else
        {
          ROS_INFO("AtlasSimInterface: set demo1 mode failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.end_state.error_code =
            this->atlasSimInterface->set_desired_behavior("Walk");
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::WALK:
      {
        // make a copy of the goal trajectory
        this->stepTrajectory.resize(
          this->activeGoal.params.walk_params.size());
        std::copy(this->activeGoal.params.walk_params.begin(),
                  this->activeGoal.params.walk_params.end(),
                  this->stepTrajectory.begin());

        // check trajectory length
        if (this->stepTrajectory.size() == 0)
        {
          ROS_WARN("AtlasSimInterface: set multi-step mode failed, "
                   " zero trajectory length, aborting action, "
                   " switching to Stand");
          this->actionServerResult.end_state.error_code =
            this->atlasSimInterface->set_desired_behavior("Stand");
          this->activeGoal.params.behavior =
            atlas_msgs::AtlasSimInterface::STAND;
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
          break;
        }
        else if (this->stepTrajectory.size() < NUM_REQUIRED_WALK_STEPS)
        {
          /// \TODO: test this
          ROS_DEBUG("fill trajectory shorter than 4 with zeros.");
          for (unsigned int i = this->stepTrajectory.size();
               i < NUM_REQUIRED_WALK_STEPS; ++i)
          {
            atlas_msgs::AtlasBehaviorStepData step;
            this->stepTrajectory.push_back(step);
          }
        }

        // get current location of the robot in its internal
        // odometry frame
        {
          // call process_control_input once to get atlasControlOutput
          // with internal state, so we know where we are in
          // the robot's internal odometry.
          // this->actionServerResult.end_state.error_code =
          //   this->atlasSimInterface->process_control_input(
          //   this->atlasControlInput, this->atlasRobotState,
          //   this->atlasControlOutput);

          // get current foot placement
          this->currentPelvisPosition =
            this->ToVec3(this->atlasControlOutput.pos_est.position);
          this->currentLFootPosition =
            this->ToVec3(this->atlasControlOutput.foot_pos_est[0]);
          this->currentRFootPosition =
            this->ToVec3(this->atlasControlOutput.foot_pos_est[1]);

          // deduce current orientation from feet location relative
          // to pelvis.
          math::Vector3 dr = currentRFootPosition - currentLFootPosition;
          double currentOrientation = atan2(dr.x, -dr.y);

          // try to create current odometry pose, but ideally we would like
          // this from the controller.
          this->bdiOdometryFrame = math::Pose(this->currentPelvisPosition,
            math::Quaternion(0, 0, currentOrientation));

          // gzdbg <<   "current position pelvis["
          //       << this->currentPelvisPosition
          //       << "] current position l_foot["
          //       << this->currentLFootPosition
          //       << "] current position r_root["
          //       << this->currentRFootPosition
          //       << "] orientation [" << currentOrientation
          //       << "]\n";
        }

        // convert trajectory to local coordinate frame
        for (unsigned int i = 0; i < this->stepTrajectory.size(); ++i)
        {
          // find current orientation
          // regard current position as 0,0 and current orientation
          this->stepTrajectory[i].pose = this->ToPose(
            this->ToPose(this->stepTrajectory[i].pose) +
            this->bdiOdometryFrame);
        }

        // trajectory checks out, carry on
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Walk");

        // AtlasBehaviorWalkFeedback *fb =
        //   &this->atlasControlOutput.behavior_feedback.walk_feedback;
        // gzerr << " err [" << this->actionServerResult.end_state.error_code
        //       << "] fbcsi[" << fb->current_step_index
        //       << "] fbnsi[" << fb->next_step_index_needed
        //       << "] trem[" << fb->t_step_rem
        //       << "\n";

        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: starting multi-step mode.");
          // set goal's use_demo_walk to false
          AtlasBehaviorWalkParams* walkParams =
            &this->atlasControlInput.walk_params;
          walkParams->use_demo_walk = false;

          // initialize buffer with first 4 steps
          for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS;
               ++stepId)
          {
            walkParams->step_data[stepId].step_index =
              this->stepTrajectory[stepId].step_index;
            walkParams->step_data[stepId].foot_index =
              this->stepTrajectory[stepId].foot_index;
            walkParams->step_data[stepId].duration =
              this->stepTrajectory[stepId].duration;
            walkParams->step_data[stepId].position =
              this->ToVec3(this->stepTrajectory[stepId].pose.position);
            walkParams->step_data[stepId].yaw = this->ToPose(
              this->stepTrajectory[stepId].pose).rot.GetAsEuler().z;

            // gzdbg << "  building stepId : " << stepId
            //       << "  step_index["
            //       << walkParams->step_data[stepId].step_index
            //       << "]  isRight["
            //       << this->stepTrajectory[stepId].foot_index
            //       << "]  pos ["
            //       << walkParams->step_data[stepId].position.n[0]
            //       << "]\n";
          }
        }
        else
        {
          ROS_INFO("AtlasSimInterface: set multi-step mode failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.end_state.error_code =
            this->atlasSimInterface->set_desired_behavior("Walk");
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::DEMO2:
      {
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Walk");
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: starting DEMO2 (figure 8 pattern).");
          // set goal's use_demo_walk to true
          AtlasBehaviorWalkParams* walkParams =
            &this->atlasControlInput.walk_params;
          walkParams->use_demo_walk = true;

        }
        else
        {
          ROS_INFO("AtlasSimInterface: set multi-step mode failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::STEP:
      {
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("step");
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: starting step mode.");
          ROS_WARN(" Single step mode (not yet implemented).");
        }
        else
        {
          ROS_INFO("AtlasSimInterface: set step mode failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::STAND:
      {
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->set_desired_behavior("Stand");
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: starting Stand mode.");
          this->actionServer->setSucceeded(this->actionServerResult);
        }
        else
        {
          ROS_INFO("AtlasSimInterface: set Stand mode failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    case atlas_msgs::AtlasSimInterface::RESET:
      {
        this->ZeroAtlasCommand();
        this->actionServerResult.end_state.error_code =
          this->atlasSimInterface->reset_control();
        if (this->actionServerResult.end_state.error_code == NO_ERRORS)
        {
          ROS_INFO("AtlasSimInterface: reset successful.");
          this->actionServer->setSucceeded(this->actionServerResult);
        }
        else
        {
          ROS_INFO("AtlasSimInterface: reset failed, error code "
                   "(%d).", this->actionServerResult.end_state.error_code);
          this->actionServerResult.success = false;
          this->actionServer->setAborted(this->actionServerResult);
        }
      }
      break;
    default:
        ROS_WARN("Walking param mode not recognized, see "
                 "atlas_msgs/AtlasSimInterface.msg for options.");
      break;
  }
}



  // good set of initial values
  static const double strideSagittalDefault    = 0.25;
  static const double strideCoronalDefault     = 0.15;
  static const double stepWidthDefault         = 0.12;
  static const double strideDurationDefault    = 0.63;
  static const double walkYawRateDefault       = 0.1;

  this->strideSagittal    = strideSagittalDefault;
  this->strideCoronal     = strideCoronalDefault;
  this->stepWidth         = stepWidthDefault;
  this->strideDuration    = strideDurationDefault;
  this->walkYawRate       = walkYawRateDefault;

  // actionlib simple action server
  this->actionServer = new ActionServer(*this->rosNode, "atlas/bdi_control",
    false);

  this->actionServer->registerGoalCallback(
    boost::bind(&AtlasPlugin::ActionServerCallback, this));

  this->actionServer->start();

  // default controller state is PID only, AtlasSimInterface off by default.
  this->activeGoal.params.behavior = atlas_msgs::AtlasSimInterface::USER;
      else switch (this->activeGoal.params.behavior)
      {
        case atlas_msgs::AtlasSimInterface::FREEZE:
        case atlas_msgs::AtlasSimInterface::STAND_PREP:
        case atlas_msgs::AtlasSimInterface::STAND:
          {
            // process data atlasRobotState to get atlasControlOutput
            this->actionServerResult.end_state.error_code =
              this->atlasSimInterface->process_control_input(
              this->atlasControlInput, this->atlasRobotState,
              this->atlasControlOutput);

            if (this->actionServerResult.end_state.error_code == NO_ERRORS)
            {
              // stuff data from atlasControlOutput into actionServerFeedback.
              this->UpdateActionServerStateFeedback();
            }
            else
            {
              ROS_WARN("AtlasSimInterface: failed with error code (%d) when "
                       "setting mode (%d) in update loop. Switching to "
                       "Stand mode",
                       this->actionServerResult.end_state.error_code,
                       this->activeGoal.params.behavior);
              this->actionServerResult.success = false;
              this->actionServer->setAborted(this->actionServerResult);
            }
          }
          break;
        case atlas_msgs::AtlasSimInterface::DEMO2:
          {
            // process data atlasRobotState to get atlasControlOutput
            this->actionServerResult.end_state.error_code =
              this->atlasSimInterface->process_control_input(
              this->atlasControlInput, this->atlasRobotState,
              this->atlasControlOutput);

            // stuff data from atlasControlOutput into actionServerFeedback.
            this->UpdateActionServerStateFeedback();

            // save typing
            AtlasBehaviorWalkFeedback *fb =
              &this->atlasControlOutput.behavior_feedback.walk_feedback;

            // gzdbg << fb->current_step_index << " : "
            //       << this->actionServerResult.end_state.error_code
            //       << "\n";

            /// \TODO: detect when demo finishes, switch to Stand and end action
            static const int demo2Steps = 47;
            if (fb->current_step_index >= demo2Steps)
            {
              ROS_INFO("AtlasSimInterface: demo2 finished, going into Stand.");
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STAND;
              this->actionServerResult.end_state.error_code =
                this->atlasSimInterface->set_desired_behavior("Stand");
              this->actionServerResult.success = true;
              this->actionServer->setSucceeded(this->actionServerResult);
            }
          }
          break;
        case atlas_msgs::AtlasSimInterface::DEMO1:
          {
            // DEMO1 Update
            // process data atlasRobotState to get atlasControlOutput
            this->actionServerResult.end_state.error_code =
              this->atlasSimInterface->process_control_input(
              this->atlasControlInput, this->atlasRobotState,
              this->atlasControlOutput);

            // stuff data from atlasControlOutput into actionServerFeedback.
            this->UpdateActionServerStateFeedback();

            /* AtlasRobotState accessor could be used here instead.
            if (this->resetAtlasRobotState)
            {
              // call this once
              this->resetAtlasRobotState = false;

              // get current foot placement
              this->currentPelvisPosition =
                this->ToVec3(this->atlasControlOutput.pos_est.position);
              this->currentLFootPosition =
                this->ToVec3(this->atlasControlOutput.foot_pos_est[0]);
              this->currentRFootPosition =
                this->ToVec3(this->atlasControlOutput.foot_pos_est[1]);

              // deduce current orientation from feet location relative
              // to pelvis.
              math::Vector3 dr = currentRFootPosition - currentLFootPosition;
              double currentOrientation = atan2(dr.x, -dr.y);

              // try to create current odometry pose, but ideally we would like
              // this from the controller.
              this->bdiOdometryFrame = math::Pose(this->currentPelvisPosition,
                math::Quaternion(0, 0, currentOrientation));

              // gzdbg <<   "current position pelvis["
              //       << this->currentPelvisPosition
              //       << "] current position l_foot["
              //       << this->currentLFootPosition
              //       << "] current position r_root["
              //       << this->currentRFootPosition
              //       << "] orientation [" << currentOrientation
              //       << "]\n";
            }*/

            // sanity check
            if (mode != "Walk")
            {
              ROS_ERROR("DEMO1: mode is [%s], something is wrong, "
                        "switching back to Stand", mode.c_str());
              this->actionServerResult.end_state.error_code =
                this->atlasSimInterface->set_desired_behavior("Stand");
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STAND;
              this->actionServerResult.success = false;
              this->actionServer->setAborted(this->actionServerResult);
            }

            // roll out trajectory, update atlasRobotState.walk_params
            // get pointer to current walking param
            AtlasBehaviorWalkParams* walkParams =
              &this->atlasControlInput.walk_params;

            // Update trajectory buffer
            {

              AtlasBehaviorWalkFeedback *fb =
                &this->atlasControlOutput.behavior_feedback.walk_feedback;

              // gzdbg << "e [" << this->actionServerResult.end_state.error_code
              //       << "] fbcsi[" << fb->current_step_index
              //       << "] fbnsi[" << fb->next_step_index_needed
              //       << "] trem[" << fb->t_step_rem
              //       << "] ok["
              //       << (bool)(fb->status_flags & WALK_OKAY)
              //       << "] sway["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_SWAYING)
              //       << "] step["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_STEPPING)
              //       << "] catch["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_CATCHING)
              //       << "] insf["
              //       << (bool)(fb->status_flags &
              //                 WALK_WARNING_INSUFFICIENT_STEP_DATA)
              //       << "] inco["
              //       << (bool)(fb->status_flags&WALK_ERROR_INCONSISTENT_STEPS)
              //       << "]\n";

              if (walkParams->step_data[0].step_index <
                  fb->next_step_index_needed)
              {
                // cache last foot index
                bool footIndex = walkParams->step_data[0].foot_index;

                math::Pose odometryFrame = this->bdiOdometryFrame;

                // initialize buffer with first 4 steps
                for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS;
                     ++stepId)
                {
                  // switch foot
                  footIndex = !footIndex;

                  // step lateral offset in local odometry frame
                  double coronalOffset = (1.0 - 2.0*footIndex)*this->stepWidth;

                  // how much will odometry frame change, expressed in
                  // current odometry frame
                  // demo1Vel is specified in m / step or rad / step
                  math::Pose dOdometryFrame(
                    math::Vector3(this->demo1Vel.x,
                                  this->demo1Vel.y, 0.0),
                    math::Quaternion(0, 0, this->demo1Vel.z));

                  // compute next step based on update
                  odometryFrame = dOdometryFrame + odometryFrame;

                  // step location, local frame
                  math::Pose stepLocation(0, coronalOffset, 0, 0, 0, 0);

                  // step location, odometry frame
                  stepLocation = stepLocation + odometryFrame;

                  // update next step location
                  if (stepId == 0)
                    this->bdiOdometryFrame = odometryFrame;

                  // gzdbg << coronalOffset << " | " << odometryFrame << "\n";

                  walkParams->step_data[stepId].step_index =
                    stepId + fb->next_step_index_needed;
                  // alternate foot
                  walkParams->step_data[stepId].foot_index = footIndex;
                  walkParams->step_data[stepId].duration =
                    this->strideDuration;
                  walkParams->step_data[stepId].position =
                    AtlasVec3f(ToVec3(stepLocation.pos));
                  walkParams->step_data[stepId].yaw = stepLocation.rot.GetYaw();

                  gzdbg << "building stepId : " << stepId
                        << "  step_index["
                        << walkParams->step_data[stepId].step_index
                        << "]  isRight["
                        << walkParams->step_data[stepId].foot_index
                        << "]  pos ["
                        << walkParams->step_data[stepId].position.n[0]
                        << ", "
                        << walkParams->step_data[stepId].position.n[1]
                        << ", "
                        << walkParams->step_data[stepId].position.n[2]
                        << "]\n";
                }
              }
            }
          }
          break;
        case atlas_msgs::AtlasSimInterface::WALK:
          {
            // process data atlasRobotState to get atlasControlOutput
            this->actionServerResult.end_state.error_code =
              this->atlasSimInterface->process_control_input(
              this->atlasControlInput, this->atlasRobotState,
              this->atlasControlOutput);

            // stuff data from atlasControlOutput into actionServerFeedback.
            this->UpdateActionServerStateFeedback();

            // sanity check
            if (mode != "Walk")
            {
              ROS_ERROR("WALK: mode is [%s], something is wrong, "
                        "switching back to Stand", mode.c_str());
              this->actionServerResult.end_state.error_code =
                this->atlasSimInterface->set_desired_behavior("Stand");
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STAND;
              this->actionServerResult.success = false;
              this->actionServer->setAborted(this->actionServerResult);
            }

            // roll out trajectory, update atlasRobotState.walk_params
            // get pointer to current walking param
            AtlasBehaviorWalkParams* walkParams =
              &this->atlasControlInput.walk_params;

            // Update trajectory buffer
            // by unrolling from local copy of step trajectory
            {
              AtlasBehaviorWalkFeedback *fb =
                &this->atlasControlOutput.behavior_feedback.walk_feedback;
              // gzerr << "e [" << this->actionServerResult.end_state.error_code
              //       << "] fbcsi[" << fb->current_step_index
              //       << "] fbnsi[" << fb->next_step_index_needed
              //       << "] trem[" << fb->t_step_rem
              //       << "] ok["
              //       << (bool)(fb->status_flags & WALK_OKAY)
              //       << "] sway["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_SWAYING)
              //       << "] step["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_STEPPING)
              //       << "] catch["
              //       << (bool)(fb->status_flags & WALK_SUBSTATE_CATCHING)
              //       << "] insf["
              //       << (bool)(fb->status_flags &
              //                 WALK_WARNING_INSUFFICIENT_STEP_DATA)
              //       << "] inco["
              //       << (bool)(fb->status_flags&WALK_ERROR_INCONSISTENT_STEPS)
              //       << "]\n";

              if (static_cast<unsigned int>(fb->current_step_index) >=
                  this->stepTrajectory.size())
              {
                // end of trajectory
                ROS_INFO("AtlasSimInterface: multi-step mode finished "
                         " successfully, switching to Stand");
                this->actionServerResult.end_state.error_code =
                  this->atlasSimInterface->set_desired_behavior("Stand");
                this->activeGoal.params.behavior =
                  atlas_msgs::AtlasSimInterface::STAND;
                this->actionServerResult.success = true;
                this->actionServer->setSucceeded(this->actionServerResult);
                break;
              }
                 
              if (walkParams->step_data[0].step_index <
                  fb->next_step_index_needed &&
                  static_cast<unsigned int>(fb->current_step_index) <
                  this->stepTrajectory.size() - NUM_REQUIRED_WALK_STEPS)
              {
                for (unsigned stepId = 0; stepId < NUM_REQUIRED_WALK_STEPS;
                     ++stepId)
                {
                  // use up to the last trajectory provided
                  unsigned int stepTrajectoryId = std::min(
                    (int)stepId + fb->next_step_index_needed - 1,
                    (int)stepTrajectory.size() - NUM_REQUIRED_WALK_STEPS +
                    (int)stepId);

                  // walkParams->step_data[stepId].step_index =
                  //   stepTrajectoryId+1;

                  // no more steps, duplcate last step for same foot in buffer
                  while (stepTrajectoryId >= this->stepTrajectory.size())
                    stepTrajectoryId -= 2;

                  walkParams->step_data[stepId].step_index =
                    this->stepTrajectory[stepTrajectoryId].step_index;
                  // gzerr << this->stepTrajectory[stepTrajectoryId].step_index
                  //   << " : " << stepTrajectoryId << "\n";
                  walkParams->step_data[stepId].foot_index =
                    this->stepTrajectory[stepTrajectoryId].foot_index;
                  walkParams->step_data[stepId].duration =
                    this->stepTrajectory[stepTrajectoryId].duration;
                  walkParams->step_data[stepId].position = this->ToVec3(
                    this->stepTrajectory[stepTrajectoryId].pose.position);
                  math::Quaternion yawQ = this->ToPose(
                    this->stepTrajectory[stepTrajectoryId].pose).rot;
                  walkParams->step_data[stepId].yaw = yawQ.GetAsEuler().z;

                  // gzdbg << "  building stepId : " << stepId
                  //       << "  traj id [" << stepTrajectoryId
                  //       << "] step_index["
                  //       << walkParams->step_data[stepId].step_index
                  //       << "]  isRight["
                  //       << walkParams->step_data[stepId].foot_index
                  //       << "]  pos ["
                  //       << walkParams->step_data[stepId].position.n[0]
                  //       << ", "
                  //       << walkParams->step_data[stepId].position.n[1]
                  //       << "]\n";
                }
              }
            }
          }
          break;
        case atlas_msgs::AtlasSimInterface::STEP:
          {
            // process data atlasRobotState to get atlasControlOutput
            this->actionServerResult.end_state.error_code =
              this->atlasSimInterface->process_control_input(
              this->atlasControlInput, this->atlasRobotState,
              this->atlasControlOutput);

            this->UpdateActionServerStateFeedback();

            if (this->actionServerResult.end_state.error_code == NO_ERRORS)
            {
              ROS_DEBUG("AtlasSimInterface: performing step mode.");
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STEP;
            }
            else if (false)  /// \TODO: check step termination
            {
              ROS_DEBUG("AtlasSimInterface: step finished fine.");
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STAND;
              this->actionServerResult.end_state.error_code =
                this->atlasSimInterface->set_desired_behavior("Stand");
            }
            else
            {
              ROS_DEBUG("AtlasSimInterface: step failed with error "
                        "code (%d).",
                        this->actionServerResult.end_state.error_code);
              this->activeGoal.params.behavior =
                atlas_msgs::AtlasSimInterface::STAND;
              this->actionServerResult.end_state.error_code =
                this->atlasSimInterface->set_desired_behavior("Stand");
              if (this->actionServerResult.end_state.error_code != NO_ERRORS)
                ROS_DEBUG("AtlasSimInterface: step switch to Stand "
                          "failed with error code (%d)",
                          this->actionServerResult.end_state.error_code);
              this->actionServerResult.success = false;
              this->actionServer->setAborted(this->actionServerResult);
            }
          }
          break;
        default:
          // do nothing here
          break;
      }
