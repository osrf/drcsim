/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <iostream>

#include "simbicon/Controller.h"

#include "simbicon/State.h"
#include "simbicon/StateMachine.h"
#include "simbicon/TerminalCondition.h"

#define DEG2RAD (M_PI / 180.0)

using namespace std;
using namespace Eigen;

//==============================================================================
Controller::Controller()
  : mCurrentStateMachine(NULL)
{
  _buildStateMachines();

  mInitialState = Eigen::VectorXd();
}

//==============================================================================
Controller::~Controller()
{
  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    delete *it;
  }
}

//==============================================================================
void Controller::update(double _dt)
{
  // Compute control force
  mCurrentStateMachine->computeControlForce(_dt);
}

StateMachine*Controller::getCurrentState()
{
  return mCurrentStateMachine;
}

//==============================================================================
void Controller::changeStateMachine(StateMachine* _stateMachine,
                                    double _currentTime)
{
  assert(_containStateMachine(_stateMachine)
         && "_stateMachine should be in mStateMachines");

  if (mCurrentStateMachine == _stateMachine)
  {
    return;
  }

  string prevName = mCurrentStateMachine->getName();
  string nextName = _stateMachine->getName();

  // Finish current state
  mCurrentStateMachine->end(_currentTime);

  // Transite to _state
  mCurrentStateMachine = _stateMachine;
  mCurrentStateMachine->begin(_currentTime);

  std::cout << "State machine transition: from [" << prevName << "] to ["
        << nextName << "]." << endl;
}

//==============================================================================
void Controller::changeStateMachine(const string& _name, double _currentTime)
{
  // _state should be in mStates
  StateMachine* stateMachine = _findStateMachine(_name);

  assert(stateMachine != NULL && "Invaild state machine.");

  changeStateMachine(stateMachine, _currentTime);
}

//==============================================================================
void Controller::changeStateMachine(size_t _idx, double _currentTime)
{
  assert(0 <= _idx && _idx <= mStateMachines.size()
         && "Invalid index of StateMachine.");

  changeStateMachine(mStateMachines[_idx], _currentTime);
}

//==============================================================================
void Controller::keyboard(unsigned char _key, int _x, int _y,
                          double _currentTime)
{
  switch (_key)
  {
    case 'n':  // Transite to the next state manually
      mCurrentStateMachine->transiteToNextState(_currentTime);
      break;
    case '1':  // Standing controller
      changeStateMachine("standing", _currentTime);
      break;
    case '2':  // Walking in place controller
      changeStateMachine("walking in place", _currentTime);
      break;
    case '3':
      changeStateMachine("walking", _currentTime);
      break;
    case '4':
      changeStateMachine("running", _currentTime);
      break;

    default:
      break;
  }
}

//==============================================================================
void Controller::_buildStateMachines()
{
  // Standing controller
  mStateMachines.push_back(_createStandingStateMachine());

  // Walking in place controller
  mStateMachines.push_back(_createWalkingInPlaceStateMachine());

  // Walking controller
  mStateMachines.push_back(_createWalkingStateMachine());

  // Walking controller
  mStateMachines.push_back(_createRunningStateMachine());

  // Set initial (default) controller
  mCurrentStateMachine = mStateMachines[1];  // Standing controller

  // Begin the default controller
  mCurrentStateMachine->begin(0.0);
}

//==============================================================================
StateMachine* Controller::_createStandingStateMachine()
{
  StateMachine* standing = new StateMachine("standing");

  std::vector<double> positions;
  std::vector<double> velocities;

  State* standingState0 = new State(positions, velocities, "0");

  TerminalCondition* tcStanding0 = new TimerCondition(standingState0, 0.3);

  standingState0->setTerminalCondition(tcStanding0);

  standingState0->setNextState(standingState0);

  // angle b/w pelvis and torso
  standingState0->setDesiredJointPosition( 9, DEG2RAD *  15.00);
  standingState0->setDesiredJointPosition(13, DEG2RAD * -10.00);
  standingState0->setDesiredJointPosition(14, DEG2RAD * -10.00);
  // left knee
  standingState0->setDesiredJointPosition(17, DEG2RAD *  30.00);
  // right knee
  standingState0->setDesiredJointPosition(18, DEG2RAD *  30.00);
  // left ankle
  standingState0->setDesiredJointPosition(21, DEG2RAD * -16.80);
  // right ankle
  standingState0->setDesiredJointPosition(22, DEG2RAD * -16.80);

  // right ankle
  standingState0->setDesiredJointPosition(19, DEG2RAD * -90.0);
  // right ankle
  standingState0->setDesiredJointPosition(20, DEG2RAD * +90.0);

  standing->addState(standingState0);

  standing->setInitialState(standingState0);

  return standing;
}

//==============================================================================
StateMachine* Controller::_createWalkingInPlaceStateMachine()
{
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = DEG2RAD * -4.75;  // angle b/w pelvis and torso

  const double swh02  =  0.50;  // swing hip
  const double swk02  = -1.10;  // swing knee
  const double swa02  =  0.60;  // swing angle
  const double stk02  = -0.05;  // stance knee
  const double sta02  =  0.00;  // stance ankle

  const double swh13  = -0.10;  // swing hip
  const double swk13  = -0.05;  // swing knee
  const double swa13  =  0.15;  // swing angle
  const double stk13  = -0.10;  // stance knee
  const double sta13  =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking in place");

  std::vector<double> positions;
  std::vector<double> velocities;

  State* state0 = new State(positions, velocities, "0");
  State* state1 = new State(positions, velocities, "1");
  State* state2 = new State(positions, velocities, "2");
  State* state3 = new State(positions, velocities, "3");

  // index to state of the feet
  int rightFootIndex = 0;
  int leftFootIndex = 0;

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1 = new BodyContactCondition(state1, rightFootIndex);
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3 = new BodyContactCondition(state3, leftFootIndex);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state2->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state3->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state2->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state3->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh02); // right hip
  state0->setDesiredJointPosition(18, -swk02); // right knee
  state0->setDesiredJointPosition(22, -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk02); // left knee
  state0->setDesiredJointPosition(21, -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DEG2RAD * -20.00); // left arm
  state0->setDesiredJointPosition(16, DEG2RAD * +10.00); // right arm
  state0->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state0->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh13); // left hip
  state1->setDesiredJointPosition(17, -swk13); // left knee
  state1->setDesiredJointPosition(21, -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk13); // right knee
  state1->setDesiredJointPosition(22, -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DEG2RAD * +10.00); // left arm
  state1->setDesiredJointPosition(16, DEG2RAD * -20.00); // right arm
  state1->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state1->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition(13, -swh02); // left hip
  state2->setDesiredJointPosition(17, -swk02); // left knee
  state2->setDesiredJointPosition(21, -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition(18, -stk02); // right knee
  state2->setDesiredJointPosition(22, -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition(15, DEG2RAD * +10.00); // left arm
  state2->setDesiredJointPosition(16, DEG2RAD * -20.00); // right arm
  state2->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state2->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state2->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state2->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state2->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state2->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state2->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition(14, -swh13); // right hip
  state3->setDesiredJointPosition(18, -swk13); // right knee
  state3->setDesiredJointPosition(22, -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition(17, -stk13); // left knee
  state3->setDesiredJointPosition(21, -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition(15, DEG2RAD * -20.00); // left arm
  state3->setDesiredJointPosition(16, DEG2RAD * +10.00); // right arm
  state3->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state3->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state3->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state3->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state3->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state3->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state3->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);

  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createWalkingStateMachine()
{
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = DEG2RAD * -10.0;  // angle b/w pelvis and torso

  const double swh02  =  0.50;  // swing hip
  const double swk02  = -1.10;  // swing knee
  const double swa02  =  0.60;  // swing angle
  const double stk02  = -0.05;  // stance knee
  const double sta02  =  0.00;  // stance ankle

  const double swh13  = -0.10;  // swing hip
  const double swk13  = -0.05;  // swing knee
  const double swa13  =  0.15;  // swing angle
  const double stk13  = -0.10;  // stance knee
  const double sta13  =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking");

  std::vector<double> positions;
  std::vector<double> velocities;

  State* state0 = new State(positions, velocities, "0");
  State* state1 = new State(positions, velocities, "1");
  State* state2 = new State(positions, velocities, "2");
  State* state3 = new State(positions, velocities, "3");

  // index to state of the feet
  int rightFootIndex = 0;
  int leftFootIndex = 0;

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1 = new BodyContactCondition(state1, rightFootIndex);
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3 = new BodyContactCondition(state3, leftFootIndex);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state2->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state3->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state2->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state3->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh02); // right hip
  state0->setDesiredJointPosition(18, -swk02); // right knee
  state0->setDesiredJointPosition(22, -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk02); // left knee
  state0->setDesiredJointPosition(21, -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DEG2RAD * -20.00); // left arm
  state0->setDesiredJointPosition(16, DEG2RAD * +10.00); // right arm
  state0->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state0->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh13); // left hip
  state1->setDesiredJointPosition(17, -swk13); // left knee
  state1->setDesiredJointPosition(21, -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk13); // right knee
  state1->setDesiredJointPosition(22, -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DEG2RAD * +10.00); // left arm
  state1->setDesiredJointPosition(16, DEG2RAD * -20.00); // right arm
  state1->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state1->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition(13, -swh02); // left hip
  state2->setDesiredJointPosition(17, -swk02); // left knee
  state2->setDesiredJointPosition(21, -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition(18, -stk02); // right knee
  state2->setDesiredJointPosition(22, -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition(15, DEG2RAD * +10.00); // left arm
  state2->setDesiredJointPosition(16, DEG2RAD * -20.00); // right arm
  state2->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state2->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state2->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state2->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state2->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state2->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state2->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition(14, -swh13); // right hip
  state3->setDesiredJointPosition(18, -swk13); // right knee
  state3->setDesiredJointPosition(22, -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition(17, -stk13); // left knee
  state3->setDesiredJointPosition(21, -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition(15, DEG2RAD * -20.00); // left arm
  state3->setDesiredJointPosition(16, DEG2RAD * +10.00); // right arm
  state3->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state3->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state3->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state3->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state3->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state3->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state3->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);

  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createRunningStateMachine()
{
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis   = DEG2RAD * -10.0;  // angle b/w pelvis and torso

  const double swh01 =  0.50;  // swing hip
  const double swk01 = -1.10;  // swing knee
  const double swa01 =  0.60;  // swing angle
  const double stk01 = -0.05;  // stance knee
  const double sta01 =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("running");

  std::vector<double> positions;
  std::vector<double> velocities;

  State* state0 = new State(positions, velocities, "0");
  State* state1 = new State(positions, velocities, "1");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.15);
  TerminalCondition* cond1 = new TimerCondition(state1, 0.15);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);

  state0->setNextState(state1);
  state1->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DEG2RAD * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DEG2RAD * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh01); // right hip
  state0->setDesiredJointPosition(18, -swk01); // right knee
  state0->setDesiredJointPosition(22, -swa01); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk01); // left knee
  state0->setDesiredJointPosition(21, -sta01); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DEG2RAD * -45.00); // left arm
  state0->setDesiredJointPosition(16, DEG2RAD * +15.00); // right arm
  state0->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state0->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
//  state0->setDesiredJointPosition(23, DEG2RAD * +90.00); // left arm
//  state0->setDesiredJointPosition(24, DEG2RAD * +90.00); // right arm
//  state0->setDesiredJointPosition(27, DEG2RAD * +90.00); // left arm
//  state0->setDesiredJointPosition(28, DEG2RAD * -90.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh01); // left hip
  state1->setDesiredJointPosition(17, -swk01); // left knee
  state1->setDesiredJointPosition(21, -swa01); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk01); // right knee
  state1->setDesiredJointPosition(22, -sta01); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DEG2RAD * +15.00); // left arm
  state1->setDesiredJointPosition(16, DEG2RAD * -45.00); // right arm
  state1->setDesiredJointPosition(19, DEG2RAD * -80.00); // left arm
  state1->setDesiredJointPosition(20, DEG2RAD * +80.00); // right arm
//  state1->setDesiredJointPosition(23, DEG2RAD * +90.00); // left arm
//  state1->setDesiredJointPosition(24, DEG2RAD * +90.00); // right arm
//  state1->setDesiredJointPosition(27, DEG2RAD * +90.00); // left arm
//  state1->setDesiredJointPosition(28, DEG2RAD * -90.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  sm->addState(state0);
  sm->addState(state1);

  sm->setInitialState(state0);

  return sm;
}

//==============================================================================
bool Controller::_containStateMachine(StateMachine* _stateMachine)
{
  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    if (*it == _stateMachine)
      return true;
  }

  return false;
}

//==============================================================================
bool Controller::_containStateMachine(const string& _name)
{
  return _containStateMachine(_findStateMachine(_name));
}

//==============================================================================
StateMachine* Controller::_findStateMachine(const string& _name)
{
  StateMachine* stateMachine = NULL;

  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    if ((*it)->getName() == _name)
    {
      stateMachine = *it;
      break;
    }
  }

  return stateMachine;
}

