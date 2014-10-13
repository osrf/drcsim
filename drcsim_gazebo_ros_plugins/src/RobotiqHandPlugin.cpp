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

#include <string>
#include <vector>
#include <atlas_msgs/SModelRobotInput.h>
#include <atlas_msgs/SModelRobotOutput.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "drcsim_gazebo_ros_plugins/RobotiqHandPlugin.h"

// Default topic names initialization.
const std::string RobotiqHandPlugin::DefaultLeftTopicCommand =
  "/left_hand/command";
const std::string RobotiqHandPlugin::DefaultLeftTopicState =
  "/left_hand/state";
const std::string RobotiqHandPlugin::DefaultRightTopicCommand =
  "/right_hand/command";
const std::string RobotiqHandPlugin::DefaultRightTopicState =
  "/right_hand/state";

////////////////////////////////////////////////////////////////////////////////
RobotiqHandPlugin::RobotiqHandPlugin()
{
  // PID default parameters.
  for (int i = 0; i < this->NumJoints; ++i)
  {
    this->posePID[i].Init(5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
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
  std::string controlTopicName = this->DefaultLeftTopicCommand;
  std::string stateTopicName   = this->DefaultLeftTopicState;
  if (this->side == "right")
  {
    controlTopicName = this->DefaultRightTopicCommand;
    stateTopicName   = this->DefaultRightTopicState;
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
    controlTopicName = this->sdf->Get<std::string>("topic_command");

  if (this->sdf->HasElement("topic_state"))
    stateTopicName = this->sdf->Get<std::string>("topic_state");

  // Load the vector of joints.
  if (!this->FindJoints())
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
  this->pubHandleStateQueue = this->pmq.addPub<atlas_msgs::SModelRobotInput>();
  this->pubHandleState = this->rosNode->advertise<atlas_msgs::SModelRobotInput>(
    stateTopicName, 100, true);

  // Subscribe to user published handle control commands.
  ros::SubscribeOptions handleCommandSo =
    ros::SubscribeOptions::create<atlas_msgs::SModelRobotOutput>(
      controlTopicName, 100,
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
  gzlog << "Topic for sending hand commands: ["   << controlTopicName
        << "]\nTopic for receiving hand state: [" << stateTopicName
        << "]" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
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
    const atlas_msgs::SModelRobotOutput::ConstPtr &_command)
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
    const atlas_msgs::SModelRobotOutput::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Sanity check.
  if (!this->VerifyCommand(_msg))
  {
    std::cerr << "Ignoring command" << std::endl;
    return;
  }

  this->prevCommand = this->handleCommand;

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

  for (int i = 2; i < this->NumJoints; ++i)
  {
    fingersOpen = fingersOpen &&
      (this->fingerJoints[i]->GetAngle(0) <
       (this->fingerJoints[i]->GetLowerLimit(0) + tolerance));
  }

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

        this->prevCommand = this->handleCommand;

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

    // Update the hand controller.
    this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());

    // Gather robot state data and publish them.
    this->GetAndPublishHandleState();

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

  // Check fingers' speed.
  const double VelTolerance = 0.002;
  bool isMovingScissorA = this->fingerJoints[0]->GetVelocity(0) > VelTolerance;
  bool isMovingScissorB = this->fingerJoints[1]->GetVelocity(0) > VelTolerance;
  bool isMovingA = this->fingerJoints[2]->GetVelocity(0) > VelTolerance;
  bool isMovingB = this->fingerJoints[3]->GetVelocity(0) > VelTolerance;
  bool isMovingC = this->fingerJoints[4]->GetVelocity(0) > VelTolerance;


  // Check if the fingers reached their target positions.
  double pe, ie, de;
  const double PoseTolerance = 0.002;
  this->posePID[0].GetErrors(pe, ie, de);
  bool reachPositionScissorA = pe < 0.002;
  this->posePID[1].GetErrors(pe, ie, de);
  bool reachPositionScissorB = pe < 0.002;
  this->posePID[2].GetErrors(pe, ie, de);
  bool reachPositionA = pe < 0.002;
  this->posePID[3].GetErrors(pe, ie, de);
  bool reachPositionB = pe < 0.002;
  this->posePID[4].GetErrors(pe, ie, de);
  bool reachPositionC = pe < 0.002;



  // Motion status.
  if (isMovingA || isMovingB || isMovingC)
  {
    // Gripper is in motion.
    this->handleState.gSTA = 0;
  }
  else
  {
    if (reachPositionA && reachPositionB && reachPositionC)
    {
      // Gripper is stopped: All fingers reached requested position.
      this->handleState.gSTA = 3;
    }
    else if (!reachPositionA && !reachPositionB && !reachPositionC)
    {
      // Gripper is stopped: All fingers stopped before requested position.
      this->handleState.gSTA = 2;
    }
    else
    {
      // Gripper is stopped. One or two fingers stopped before requested position.
      this->handleState.gSTA = 1;
    }
  }

  // Object status.

  // gDTA.
  if (isMovingA)
  {
    // Finger A is in motion.
    this->handleState.gDTA = 0;
  }
  else
  {
    if (reachPositionA)
    {
      // Finger A is at the requestedPosition.
      this->handleState.gDTA = 3;
    }
    else if (this->handleCommand.rPRA - this->prevCommand.rPRA > 0)
    {
      // Finger A has stopped due to a contact while closing.
      this->handleState.gDTA = 2;
    }
    else
    {
      // Finger A has stopped due to a contact while opening.
      this->handleState.gDTA = 1;
    }
  }

  // gDTB.
  if (isMovingB)
  {
    // Finger B is in motion.
    this->handleState.gDTB = 0;
  }
  else
  {
    if (reachPositionB)
    {
      // Finger B is at the requestedPosition.
      this->handleState.gDTB = 3;
    }
    else if (this->handleCommand.rPRB - this->prevCommand.rPRB > 0)
    {
      // Finger B has stopped due to a contact while closing.
      this->handleState.gDTB = 2;
    }
    else
    {
      // Finger B has stopped due to a contact while opening.
      this->handleState.gDTB = 1;
    }
  }

  // gDTC.
  if (isMovingC)
  {
    // Finger C is in motion.
    this->handleState.gDTC = 0;
  }
  else
  {
    if (reachPositionC)
    {
      // Finger C is at the requestedPosition.
      this->handleState.gDTC = 3;
    }
    else if (this->handleCommand.rPRC - this->prevCommand.rPRC > 0)
    {
      // Finger C has stopped due to a contact while closing.
      this->handleState.gDTC = 2;
    }
    else
    {
      // Finger C has stopped due to a contact while opening.
      this->handleState.gDTC = 1;
    }
  }

  // gDTS.
  if (isMovingScissorA || isMovingScissorB)
  {
    // Scissor is in motion.
    this->handleState.gDTS = 0;
  }
  else
  {
    if (reachPositionScissorA && reachPositionScissorB)
    {
      // Scissor is at the requestedPosition.
      this->handleState.gDTS = 3;
    }
    else if (this->handleCommand.rPRS - this->prevCommand.rPRS > 0)
    {
      // Finger C has stopped due to a contact while closing.
      this->handleState.gDTS = 2;
    }
    else
    {
      // Finger C has stopped due to a contact while opening.
      this->handleState.gDTS = 1;
    }
  }

  if (this->handState == ChangeModeInProgress)
    this->handleState.gFLT = 6;
  else if (this->handState == Disabled)
    this->handleState.gFLT = 7;
  else if (this->handState == Emergency)
    this->handleState.gFLT = 11;
  else
    this->handleState.gFLT = 0;

  // Echo of requested position for finger A.
  this->handleState.gPRA = this->userHandleCommand.rPRA;
  // Finger A position [0-255].
  gazebo::math::Angle range = this->fingerJoints[2]->GetUpperLimit(0) -
    this->fingerJoints[2]->GetLowerLimit(0);
  gazebo::math::Angle relAngle = this->fingerJoints[2]->GetAngle(0) -
    this->fingerJoints[2]->GetLowerLimit(0);
  this->handleState.gPOA =
    static_cast<int>(round(255.0 * relAngle.Radian() / range.Radian()));
  // Not implemented.
  this->handleState.gCUA = 0;


  // Echo of requested position for finger B.
  this->handleState.gPRB = this->userHandleCommand.rPRB;
  // Finger B position [0-255].
  range = this->fingerJoints[3]->GetUpperLimit(0) -
    this->fingerJoints[3]->GetLowerLimit(0);
  relAngle = this->fingerJoints[3]->GetAngle(0) -
    this->fingerJoints[3]->GetLowerLimit(0);
  this->handleState.gPOB =
    static_cast<int>(round(255.0 * relAngle.Radian() / range.Radian()));
  // Not implemented.
  this->handleState.gCUB = 0;

  // Echo of requested position for finger C.
  this->handleState.gPRC = this->userHandleCommand.rPRC;
  // Finger C position [0-255].
  range = this->fingerJoints[4]->GetUpperLimit(0) -
    this->fingerJoints[4]->GetLowerLimit(0);
  relAngle = this->fingerJoints[4]->GetAngle(0) -
    this->fingerJoints[4]->GetLowerLimit(0);
  this->handleState.gPOC =
    static_cast<int>(round(255.0 * relAngle.Radian() / range.Radian()));
  // Not implemented.
  this->handleState.gCUC = 0;

  // Echo of requested position of the scissor action
  this->handleState.gPRS = this->userHandleCommand.rPRS;
  // Scissor current position [0-255]. We use the angle of finger B as reference
  range = this->fingerJoints[1]->GetUpperLimit(0) -
    this->fingerJoints[1]->GetLowerLimit(0);
  relAngle = this->fingerJoints[1]->GetAngle(0) -
    this->fingerJoints[1]->GetLowerLimit(0);
  this->handleState.gPOS =
    static_cast<int>(round(255.0 * relAngle.Radian() / range.Radian()));
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
    gzerr << "Failed to find joint [" << _jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  _joints.push_back(joint);
  gzlog << "RobotiqHandPlugin found joint [" << _jointName << "]" << std::endl;
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
