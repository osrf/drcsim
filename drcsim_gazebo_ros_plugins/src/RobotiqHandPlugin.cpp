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

////////////////////////////////////////////////////////////////////////////////
RobotiqHandPlugin::RobotiqHandPlugin()
{
  this->errorTerms.resize(this->NumJoints);

  // PID default parameters.
  for (int i = 0; i < this->NumJoints; ++i)
  {
  	this->kpPosition[i]   = 1.0;
  	this->kiPosition[i]   = 0.0;
  	this->kdPosition[i]   = 0.0;
  	this->posEffortMin[i] = 0.0;
  	this->posEffortMax[i] = 0.0;
  	this->kpVelocity[i]   = 0.1;
  	this->kiVelocity[i]   = 0.0;
  	this->kdVelocity[i]   = 0.0;
  	this->velEffortMin[i] = 0.0;
  	this->velEffortMax[i] = 0.0;

    this->errorTerms[i].q_p      = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i      = 0;
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
	           "aborting plugin load. Side should be either 'left' or 'right'."
          << std::endl;
	  return;
  }

  // Overload the PID parameters if they are available.
  for (int i = 0; i < this->NumJoints; ++i)
  {
    if (this->sdf->HasElement("kp_position"))
      this->kpPosition[i] = this->sdf->Get<double>("kp_position");

    if (this->sdf->HasElement("ki_position"))
      this->kiPosition[i] = this->sdf->Get<double>("ki_position");

    if (this->sdf->HasElement("kd_position"))
      this->kdPosition[i] = this->sdf->Get<double>("kd_position");

    if (this->sdf->HasElement("position_effort_min"))
      this->posEffortMin[i] = this->sdf->Get<double>("position_effort_min");

    if (this->sdf->HasElement("position_effort_max"))
      this->posEffortMax[i] = this->sdf->Get<double>("position_effort_max");

    if (this->sdf->HasElement("kp_velocity"))
      this->kpVelocity[i] = this->sdf->Get<double>("kp_velocity");

    if (this->sdf->HasElement("ki_velocity"))
      this->kiVelocity[i] = this->sdf->Get<double>("ki_velocity");

    if (this->sdf->HasElement("kd_velocity"))
      this->kdVelocity[i] = this->sdf->Get<double>("kd_velocity");

    if (this->sdf->HasElement("velocity_effort_min"))
      this->velEffortMin[i] = this->sdf->Get<double>("velocity_effort_min");

    if (this->sdf->HasElement("velocity_effort_max"))
      this->velEffortMax[i] = this->sdf->Get<double>("velocity_effort_max");
  }

  gzlog << "RobotiqHandPlugin loading for " << this->side
        << " hand." << std::endl;

  // Load the vector of joints.
  if(!this->FindJoints())
	  return;

  // Initialize ROS.
  if (!ros::isInitialized())
  {
	  gzerr << "Not loading plugin since ROS hasn't been "
	        << "properly initialized.  Try starting gazebo with ros plugin:\n"
	        << "  gazebo -s libgazebo_ros_api_plugin.so\n";
	  return;
  }

  // Create a ROS node.
  this->rosNode.reset(new ros::NodeHandle(""));

  // Publish multi queue.
  this->pmq.startServiceThread();

  std::string sensorStr =  "/left_hand/state";
  std::string controlStr = "/left_hand/command";
  if (this->side != "left") {
	  sensorStr =  "/right_hand/state";
	  controlStr = "/right_hand/command";
  }

  // Broadcasts state.
  this->pubHandleStateQueue =
    this->pmq.addPub<robotiq_s_model_control::SModel_robot_input>();
  this->pubHandleState =
    this->rosNode->advertise<robotiq_s_model_control::SModel_robot_input>(
      sensorStr, 100, true);

  // Subscribe to user published handle control commands.
  ros::SubscribeOptions handleCommandSo =
	  ros::SubscribeOptions::create<robotiq_s_model_control::SModel_robot_output>(
	    controlStr, 100,
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
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::VerifyCommand(
    const robotiq_s_model_control::SModel_robot_output::ConstPtr &_command)
{
  bool res = true;
  if (_command->rACT < 0 || _command->rACT > 1)
  {
    std::cerr << "Illegal rACT value: [" << _command->rACT << "]. The correct "
              << "range is [0-1]" << std::endl;
    res = false;
  }

  if (_command->rMOD < 0 || _command->rMOD > 3)
  {
    std::cerr << "Illegal rMOD value: [" << _command->rMOD << "]. The correct "
              << "range is [0-3]" << std::endl;
    res = false;
  }

  if (_command->rGTO < 0 || _command->rGTO > 1)
  {
    std::cerr << "Illegal rGTO value: [" << _command->rGTO << "]. The correct "
              << "range is [0-1]" << std::endl;
    res = false;
  }

  if (_command->rATR < 0 || _command->rATR > 1)
  {
    std::cerr << "Illegal rATR value: [" << _command->rATR << "]. The correct "
              << "range is [0-1]" << std::endl;
    res = false;
  }

  if (_command->rICF < 0 || _command->rICF > 1)
  {
    std::cerr << "Illegal rICF value: [" << _command->rICF << "]. The correct "
              << "range is [0-1]" << std::endl;
    res = false;
  }

  if (_command->rICS < 0 || _command->rICS > 1)
  {
    std::cerr << "Illegal rICS value: [" << _command->rICS << "]. The correct "
              << "range is [0-1]" << std::endl;
    res = false;
  }

  if (_command->rPRA < 0 || _command->rPRA > 255)
  {
    std::cerr << "Illegal rPRA value: [" << _command->rPRA << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rSPA < 0 || _command->rSPA > 255)
  {
    std::cerr << "Illegal rSPA value: [" << _command->rSPA << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rFRA < 0 || _command->rFRA > 255)
  {
    std::cerr << "Illegal rFRA value: [" << _command->rFRA << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rPRB < 0 || _command->rPRB > 255)
  {
    std::cerr << "Illegal rPRB value: [" << _command->rPRB << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rSPB < 0 || _command->rSPB > 255)
  {
    std::cerr << "Illegal rSPB value: [" << _command->rSPB << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rFRB < 0 || _command->rFRB > 255)
  {
    std::cerr << "Illegal rFRB value: [" << _command->rFRB << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rPRC < 0 || _command->rPRC > 255)
  {
    std::cerr << "Illegal rPRC value: [" << _command->rPRC << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rSPC < 0 || _command->rSPC > 255)
  {
    std::cerr << "Illegal rSPC value: [" << _command->rSPC << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rFRC < 0 || _command->rFRC > 255)
  {
    std::cerr << "Illegal rFRC value: [" << _command->rFRC << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rPRS < 0 || _command->rPRS > 255)
  {
    std::cerr << "Illegal rPRS value: [" << _command->rPRS << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rSPS < 0 || _command->rSPS > 255)
  {
    std::cerr << "Illegal rSPS value: [" << _command->rSPS << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  if (_command->rFRS < 0 || _command->rFRS > 255)
  {
    std::cerr << "Illegal rFRS value: [" << _command->rFRS << "]. The correct "
              << "range is [0-255]" << std::endl;
    res = false;
  }

  return res;
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

  for (int i = 3; i < 6; ++i)
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

  for (int j = 0; j < 6; j++)
  {
  	double torque;
  	double target;
    double speed = 1.0;

    if (j == 0)
    {
      switch (this->graspingMode)
      {
        case Basic:
          target = 0.0;
          break;

        case Wide:
          target = this->fingerJoints[j]->GetUpperLimit(0).Radian();
          break;

        case Pinch:
          target = this->fingerJoints[j]->GetLowerLimit(0).Radian();
          break;

        case Scissor:
          target = this->fingerJoints[j]->GetUpperLimit(0).Radian() -
            (this->fingerJoints[j]->GetUpperLimit(0).Radian() -
             this->fingerJoints[j]->GetLowerLimit(0).Radian())
            * this->handleCommand.rPRA / 255.0;
          break;
      }
      speed = 0.5;
    }
    else if (j == 1)
    {
      switch (this->graspingMode)
      {
        case Basic:
          target = 0.0;
          break;

        case Wide:
          target = this->fingerJoints[j]->GetLowerLimit(0).Radian();
          break;

        case Pinch:
          target = this->fingerJoints[j]->GetUpperLimit(0).Radian();
          break;

        case Scissor:
          target = this->fingerJoints[j]->GetLowerLimit(0).Radian() +
            (this->fingerJoints[j]->GetUpperLimit(0).Radian() -
             this->fingerJoints[j]->GetLowerLimit(0).Radian())
            * this->handleCommand.rPRA / 255.0;
          break;
      }
      speed = 0.5;
    }
    else if (j == 2)
    {
      target = 0.0;
      speed = 0.5;
    }
	  else if (j == 3)
    {
      if (this->graspingMode == Scissor)
        target = 0.0;
      else
      {
	      //target = this->handleCommand.rPRA * 0.006;
        target = this->fingerJoints[j]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[j]->GetUpperLimit(0).Radian() -
           this->fingerJoints[j]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRA / 255.0;
      }
      speed = this->handleCommand.rSPA / 255.0;
	  }
	  else if (j == 4)
    {
      if (this->graspingMode == Scissor)
        target = 0.0;
      else
      {
        // target = this->handleCommand.rPRB * 0.006;
        target = this->fingerJoints[j]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[j]->GetUpperLimit(0).Radian() -
           this->fingerJoints[j]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRB / 255.0;
      }
      speed = this->handleCommand.rSPB / 255.0;
	  }
	  else if (j == 5)
    {
     if (this->graspingMode == Scissor)
        target = 0.0;
      else
      {
        //target = this->handleCommand.rPRC * 0.006;
        target = this->fingerJoints[j]->GetLowerLimit(0).Radian() +
          (this->fingerJoints[j]->GetUpperLimit(0).Radian() -
           this->fingerJoints[j]->GetLowerLimit(0).Radian())
          * this->handleCommand.rPRC / 255.0;
      }
      speed = this->handleCommand.rSPC / 255.0;
	  }

    // Speed multiplier.
    speed *= 2.0;

	  double current;
	  double baseJointPos = 0;
	  double baseJointVel = 0;
	  double flexureFlexJointPos = 0;
	  double flexureFlexJointVel = 0;
	  double currentPos = 0;
	  double currentVel = 0;

	  baseJointPos = this->fingerJoints[j]->GetAngle(0).Radian();
	  baseJointVel = this->fingerJoints[j]->GetVelocity(0);
	  currentPos = baseJointPos;
	  currentVel = baseJointVel;

	  double kp, ki, kd, effortMin, effortMax;

	  // Set state for position control.
	  current = currentPos;

    // Position PID.
	  kp = this->kpPosition[j] * speed;
	  ki = this->kiPosition[j] * speed;
	  kd = this->kdPosition[j] * speed;
	  effortMin = this->posEffortMin[j];
	  effortMax = this->posEffortMax[j];

	  double qP = target - current;

	  this->errorTerms[j].q_p = qP;
	  if (!gazebo::math::equal(_dt, 0.0))
    {
	    this->errorTerms[j].d_q_p_dt = (qP - this->errorTerms[j].q_p) / _dt;
	  }
	  this->errorTerms[j].q_i = gazebo::math::clamp(
	    this->errorTerms[j].q_i + _dt * this->errorTerms[j].q_p,
	    effortMin, effortMax);

	  // Use gain params to compute force cmd.
	  torque = kp * this->errorTerms[j].q_p +
	    ki * this->errorTerms[j].q_i +
	    kd * this->errorTerms[j].d_q_p_dt;

	  this->fingerJoints[j]->SetForce(0, torque);
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
  if (!this->GetAndPushBackJoint(prefix + "_palm_finger_middle_joint",
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
