/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <ros/ros.h>
#include "drcsim_gazebo_ros_plugins/RobotiqHandPlugin.h"

// Default topic names initialization.
const std::string RobotiqHandPlugin::LeftTopicCommand = "/left_hand/command";
const std::string RobotiqHandPlugin::LeftTopicState = "/left_hand/state";
const std::string RobotiqHandPlugin::RightTopicCommand = "/right_hand/command";
const std::string RobotiqHandPlugin::RightTopicState = "/right_hand/state";

////////////////////////////////////////////////////////////////////////////////
RobotiqHandPlugin::RobotiqHandPlugin()
{
  // PID default parameters.
  for (int i = 0; i < this->NumJoints; ++i)
  {
    this->posePID[i].Init(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    this->posePID[i].SetCmd(0.0);
  }

  // Default grasping mode: Basic mode.
  this->graspingMode = Basic;

  // Default hand state: Disabled.
  this->handState = Disabled;
}

////////////////////////////////////////////////////////////////////////////////
RobotiqHandPlugin::~RobotiqHandPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->sdf = _sdf;

  if (!this->sdf->HasElement("side") ||
      !this->sdf->GetElement("side")->GetValue()->Get(this->side) ||
      ((this->side != "left") && (this->side != "right")))
  {
	  gzerr << "Failed to determine which hand we're controlling; "
	           "aborting plugin load. <Side> should be either 'left' or 'right'."
          << std::endl;
	  return;
  }

  // Default ROS topic names.
  std::string controlTopicName = this->LeftTopicCommand;
  std::string stateTopicName   = this->LeftTopicState;
  if (this->side == "right")
  {
    controlTopicName = this->RightTopicCommand;
    stateTopicName   = this->RightTopicState;
  }

  // Overload the PID parameters if they are available.
  for (int i = 0; i < this->NumJoints; ++i)
  {
    if (this->sdf->HasElement("kp_position"))
      this->posePID[i].SetPGain(this->sdf->Get<double>("kp_position"));

    if (this->sdf->HasElement("ki_position"))
      this->posePID[i].SetIGain(this->sdf->Get<double>("ki_position"));

    if (this->sdf->HasElement("kd_position"))
      this->posePID[i].SetDGain(this->sdf->Get<double>("kd_position"));

    if (this->sdf->HasElement("position_effort_min"))
      this->posePID[i].SetCmdMin(this->sdf->Get<double>("position_effort_min"));

    if (this->sdf->HasElement("position_effort_max"))
      this->posePID[i].SetCmdMax(this->sdf->Get<double>("position_effort_max"));
  }

  // Overload the ROS topics for the hand if they are available.
  if (this->sdf->HasElement("topic_command"))
    this->controlTopicName = this->sdf->Get<std::string>("topic_command");

  if (this->sdf->HasElement("topic_state"))
    this->stateTopicName = this->sdf->Get<std::string>("topic_state");

  // Load the vector of joints.
  if(!this->FindJoints())
	  return;

  // Initialize ROS.
  if (!ros::isInitialized())
  {
	  gzerr << "Not loading plugin since ROS hasn't been "
	        << "properly initialized. Try starting gazebo with ros plugin:\n"
	        << " gazebo -s libgazebo_ros_api_plugin.so\n";
	  return;
  }

  // Create a ROS node.
  this->rosNode.reset(new ros::NodeHandle(""));

  // Publish multi queue.
  this->pmq.startServiceThread();

  // Broadcasts state.
  this->pubHandleStateQueue =
    this->pmq.addPub<robotiq_s_model_control::SModel_robot_input>();
  this->pubHandleState =
    this->rosNode->advertise<robotiq_s_model_control::SModel_robot_input>(
      this->stateTopicName, 100, true);

  // Subscribe to user published handle control commands.
  ros::SubscribeOptions handleCommandSo =
	  ros::SubscribeOptions::create<robotiq_s_model_control::SModel_robot_output>(
	    this->controlTopicName, 100,
	    boost::bind(&RobotiqHandPlugin::SetHandleCommand, this, _1),
	    ros::VoidPtr(), &this->rosQueue);

  // Enable TCP_NODELAY since TCP causes bursty communication with high jitter.
  handleCommandSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  this->subHandleCommand = this->rosNode->subscribe(handleCommandSo);

  // Controller time control.
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // Start callback queue.
  this->callbackQueueThread =
    boost::thread(boost::bind(&RobotiqHandPlugin::RosQueueThread, this));

  // Connect to gazebo world update.
  this->updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RobotiqHandPlugin::UpdateStates, this));

  // Log information.
  gzlog << "RobotiqHandPlugin loaded for " << this->side << " hand."
        << std::endl;
  for (int i = 0; i < this->NumJoints; ++i)
  {
    gzlog << "Position PID parameters for joint ["
          << this->fingerJoints[i]->GetName() << "]:"     << std::endl
          << "\tKP: "     << this->posePID[i].GetPGain()  << std::endl
          << "\tKI: "     << this->posePID[i].GetIGain()  << std::endl
          << "\tKD: "     << this->posePID[i].GetDGain()  << std::endl
          << "\tIMin: "   << this->posePID[i].GetIMin()   << std::endl
          << "\tIMax: "   << this->posePID[i].GetIMax()   << std::endl
          << "\tCmdMin: " << this->posePID[i].GetCmdMin() << std::endl
          << "\tCmdMax: " << this->posePID[i].GetCmdMax() << std::endl
          << std::endl;
  }
  gzlog << "Topic for sending hand commands: ["   << this->controlTopicName
        << "]\nTopic for receiving hand state: [" << this->stateTopicName
        << "]" << std::endl;
}

bool RobotiqHandPlugin::VerifyField(const std::string &_label, int _min,
  int _max, int _v)
{
  if (_v < _min || _v > _max)
  {
    std::cerr << "Illegal " << _label << " value: [" << _v << "]. The correct "
              << "range is [" << _min << "-" << _max << "]" << std::endl;
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::VerifyCommand(
    const robotiq_s_model_control::SModel_robot_output::ConstPtr &_command)
{
  return this->VerifyField("rACT", 0, 1, _command->rACT)   &&
         this->VerifyField("rMOD", 0, 3, _command->rACT)   &&
         this->VerifyField("rGTO", 0, 1, _command->rACT)   &&
         this->VerifyField("rATR", 0, 1, _command->rACT)   &&
         this->VerifyField("rICF", 0, 1, _command->rACT)   &&
         this->VerifyField("rICS", 0, 1, _command->rACT)   &&
         this->VerifyField("rPRA", 0, 255, _command->rACT) &&
         this->VerifyField("rSPA", 0, 255, _command->rACT) &&
         this->VerifyField("rFRA", 0, 255, _command->rACT) &&
         this->VerifyField("rPRB", 0, 255, _command->rACT) &&
         this->VerifyField("rSPB", 0, 255, _command->rACT) &&
         this->VerifyField("rFRB", 0, 255, _command->rACT) &&
         this->VerifyField("rPRC", 0, 255, _command->rACT) &&
         this->VerifyField("rSPC", 0, 255, _command->rACT) &&
         this->VerifyField("rFRC", 0, 255, _command->rACT) &&
         this->VerifyField("rPRS", 0, 255, _command->rACT) &&
         this->VerifyField("rSPS", 0, 255, _command->rACT) &&
         this->VerifyField("rFRS", 0, 255, _command->rACT);
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::SetHandleCommand(
    const robotiq_s_model_control::SModel_robot_output::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Sanity check.
  if (!this->VerifyCommand(_msg))
  {
    std::cerr << "Ignoring command" << std::endl;
    return;
  }

  // Update handleCommand.
  this->handleCommand = *_msg;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::ReleaseHand()
{
  // Open the fingers.
  this->handleCommand.rPRA = 0;
  this->handleCommand.rPRB = 0;
  this->handleCommand.rPRC = 0;

  // Half speed.
  this->handleCommand.rSPA = 127;
  this->handleCommand.rSPB = 127;
  this->handleCommand.rSPC = 127;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::StopHand()
{
  // Set the target positions to the current ones.
  this->handleCommand.rPRA = this->handleState.gPRA;
  this->handleCommand.rPRB = this->handleState.gPRB;
  this->handleCommand.rPRC = this->handleState.gPRC;
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::IsHandFullyOpen()
{
  bool fingersOpen = true;

  // The hand will be fully open when all the fingers are within 'tolerance'
  // from their lower limits.
  gazebo::math::Angle tolerance;
  tolerance.SetFromDegree(1.0);

  for (int i = 2; i < 5; ++i)
    fingersOpen = fingersOpen &&
      (this->fingerJoints[i]->GetAngle(0) <
       (this->fingerJoints[i]->GetLowerLimit(0) + tolerance));

  return fingersOpen;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::UpdateStates()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  gazebo::common::Time curTime = this->world->GetSimTime();

  // Step 1: State transitions.
  if (curTime > this->lastControllerUpdateTime)
  {
    this->userHandleCommand = this->handleCommand;

    // Deactivate gripper.
    if (this->handleCommand.rACT == 0)
    {
      this->handState = Disabled;
    }
    // Emergency auto-release.
    else if (this->handleCommand.rATR == 1)
    {
      this->handState = Emergency;
    }
    // Individual Control of Scissor.
    else if (this->handleCommand.rICS == 1)
    {
      this->handState = ICS;
    }
    // Individual Control of Fingers.
    else if (this->handleCommand.rICF == 1)
    {
      this->handState = ICF;
    }
    else
    {
      // Change the grasping mode.
      if (static_cast<int>(this->handleCommand.rMOD) != this->graspingMode)
      {
        this->handState = ChangeModeInProgress;
        lastHandleCommand = handleCommand;

        // Update the grasping mode.
        this->graspingMode =
          static_cast<GraspingMode>(this->handleCommand.rMOD);
      }
      else if (this->handState != ChangeModeInProgress)
      {
        this->handState = Simplified;
      }

      // Grasping mode initialized, let's change the state to Simplified Mode.
      if (this->handState == ChangeModeInProgress && this->IsHandFullyOpen())
      {
        // Restore the original command.
        this->handleCommand = this->lastHandleCommand;
        this->handState = Simplified;
      }
    }

    // Step 2: Actions in each state.
    switch (this->handState)
    {
      case Disabled:
        break;

      case Emergency:
        // Open the hand.
        if (this->IsHandFullyOpen())
          this->StopHand();
        else
          this->ReleaseHand();
        break;

      case ICS:
        std::cerr << "Individual Control of Scissor not supported" << std::endl;
        break;

      case ICF:
        if (this->handleCommand.rGTO == 0)
        {
          // "Stop" action.
          this->StopHand();
        }
        break;

      case ChangeModeInProgress:
        // Open the hand.
        this->ReleaseHand();
        break;

      case Simplified:
        // We are in Simplified mode, so all the fingers should follow finger A.
        // Position.
        this->handleCommand.rPRB = this->handleCommand.rPRA;
        this->handleCommand.rPRC = this->handleCommand.rPRA;
        // Velocity.
        this->handleCommand.rSPB = this->handleCommand.rSPA;
        this->handleCommand.rSPC = this->handleCommand.rSPA;
        // Force.
        this->handleCommand.rFRB = this->handleCommand.rFRA;
        this->handleCommand.rFRC = this->handleCommand.rFRA;

        if (this->handleCommand.rGTO == 0)
        {
          // "Stop" action.
          this->StopHand();
        }
        break;

      default:
        std::cerr << "Unrecognized state [" << this->handState << "]"
                  << std::endl;
    }

	  // Gather robot state data and publish them.
	  this->GetAndPublishHandleState();

    // Update the hand controller.
	  this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());
	  this->lastControllerUpdateTime = curTime;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::GetAndPublishHandleState()
{
  this->handleState.gACT = this->userHandleCommand.rACT;
  this->handleState.gMOD = this->userHandleCommand.rMOD;
  this->handleState.gGTO = this->userHandleCommand.rGTO;

  if (this->handState == Emergency)
    this->handleState.gIMC = 0;
  else if (this->handState == ChangeModeInProgress)
    this->handleState.gIMC = 2;
  else
    this->handleState.gIMC = 3;

  // ToDo (caguero): Check movement of the fingers.
  this->handleState.gSTA = 0;
  // ToDo (caguero): Check movement of the fingers.
  this->handleState.gDTA = 0;
  // ToDo (caguero): Check movement of the fingers.
  this->handleState.gDTB = 0;
  // ToDo (caguero): Check movement of the fingers.
  this->handleState.gDTC = 0;
  // ToDo (caguero): Check movement of the fingers.
  this->handleState.gDTS = 0;

  if (this->handState == ChangeModeInProgress)
    this->handleState.gFLT = 6;
  else if (this->handState == Disabled)
    this->handleState.gFLT = 7;
  else if (this->handState == Emergency)
    this->handleState.gFLT = 11;
  else
    this->handleState.gFLT = 0;

  // echo of requested position for finger a
  this->handleState.gPRA = this->userHandleCommand.rPRA;
  // ToDo: Normalize position between [0-255].
  this->handleState.gPOA = 0;
  // Not implemented.
  this->handleState.gCUA = 0;

  this->handleState.gPRB = this->userHandleCommand.rPRB;
  // ToDo (caguero): Normalize position between [0-255].
  this->handleState.gPOB = 0;
  // Not implemented.
  this->handleState.gCUB = 0;

  this->handleState.gPRC = this->userHandleCommand.rPRC;
  // ToDo (caguero): Normalize position between [0-255].
  this->handleState.gPOC = 0;
  // Not implemented.
  this->handleState.gCUC = 0;


  this->handleState.gPRS = this->userHandleCommand.rPRS;
  // ToDo: Implemented scissor current position [0-255].
  this->handleState.gPOS = 0;
  // Not implemented.
  this->handleState.gCUS = 0;

  // publish robot states
  this->pubHandleStateQueue->push(this->handleState, this->pubHandleState);
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::UpdatePIDControl(double _dt)
{
  if (this->handState == Disabled)
  {
    for (int i = 0; i < this->NumJoints; ++i)
      this->fingerJoints[i]->SetForce(0, 0.0);

    return;
  }

  for (int i = 0; i < this->NumJoints; ++i)
  {
  	double target = 0.0;
    double speed = 0.5;

    if (i == 0)
    {
      switch (this->graspingMode)
      {
        case Wide:
          target = this->fingerJoints[i]->GetUpperLimit(0).Radian();
          break;

        case Pinch:
          target = this->fingerJoints[i]->GetLowerLimit(0).Radian();
          break;

        case Scissor:
          target = this->fingerJoints[i]->GetUpperLimit(0).Radian() -
            (this->fingerJoints[i]->GetUpperLimit(0).Radian() -
             this->fingerJoints[i]->GetLowerLimit(0).Radian())
            * this->handleCommand.rPRA / 255.0;
          break;
      }
    }
    else if (i == 1)
    {
      switch (this->graspingMode)
      {
        case Wide:
          target = this->fingerJoints[i]->GetLowerLimit(0).Radian();
          break;

        case Pinch:
          target = this->fingerJoints[i]->GetUpperLimit(0).Radian();
          break;

        case Scissor:
          target = this->fingerJoints[i]->GetLowerLimit(0).Radian() +
            (this->fingerJoints[i]->GetUpperLimit(0).Radian() -
             this->fingerJoints[i]->GetLowerLimit(0).Radian())
            * this->handleCommand.rPRA / 255.0;
          break;
      }
    }
	  else if (i == 2)
    {
      if (this->graspingMode != Scissor)
      {
        target = this->fingerJoints[i]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[i]->GetUpperLimit(0).Radian() -
           this->fingerJoints[i]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRA / 255.0;
      }
      speed = this->handleCommand.rSPA / 255.0;
	  }
	  else if (i == 3)
    {
      if (this->graspingMode != Scissor)
      {
        target = this->fingerJoints[i]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[i]->GetUpperLimit(0).Radian() -
           this->fingerJoints[i]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRB / 255.0;
      }
      speed = this->handleCommand.rSPB / 255.0;
	  }
	  else if (i == 4)
    {
      if (this->graspingMode != Scissor)
      {
        target = this->fingerJoints[i]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[i]->GetUpperLimit(0).Radian() -
           this->fingerJoints[i]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRC / 255.0;
      }
      speed = this->handleCommand.rSPC / 255.0;
	  }

    // Speed multiplier.
    speed *= 2.0;

    // Get the current pose.
	  double current = this->fingerJoints[i]->GetAngle(0).Radian();

    // Error pose.
	  double poseError = target - current;

    // Update the PID.
    double torque = this->posePID[i].Update(poseError, _dt);

    // Apply the PID command.
	  this->fingerJoints[i]->SetForce(0, -torque);
  }
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::GetAndPushBackJoint(const std::string& _jointName,
					                                  gazebo::physics::Joint_V& _joints)
{
  gazebo::physics::JointPtr joint = this->model->GetJoint(_jointName);

  if (!joint)
  {
	  gzerr << "Failed to find joint: " << _jointName
	        << "; aborting plugin load." << std::endl;
	  return false;
  }
  _joints.push_back(joint);
  gzlog << "RobotiqHandPlugin found joint: " << _jointName << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::FindJoints()
{
  // Load up the joints we expect to use, finger by finger.
  gazebo::physics::JointPtr joint;
  std::string prefix;
  if (this->side == "left")
    prefix = "l";
  else
    prefix = "r";

  if (!this->GetAndPushBackJoint(prefix + "_palm_finger_1_joint",
        this->fingerJoints))
  {
    return false;
  }
  if (!this->GetAndPushBackJoint(prefix + "_palm_finger_2_joint",
        this->fingerJoints))
  {
    return false;
  }
	if (!this->GetAndPushBackJoint(prefix + "_finger_1_joint_1",
        this->fingerJoints))
  {
	  return false;
	}
	if (!this->GetAndPushBackJoint(prefix + "_finger_2_joint_1",
         this->fingerJoints))
  {
	  return false;
	}
	if (!this->GetAndPushBackJoint(prefix + "_finger_middle_joint_1",
          this->fingerJoints))
  {
	  return false;
 	}

  gzlog << "RobotiqHandPlugin found all joints for " << this->side
        << " hand." << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
	  this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(RobotiqHandPlugin)
