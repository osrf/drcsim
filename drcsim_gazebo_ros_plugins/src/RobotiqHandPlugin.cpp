/**
 ********************************************************************************************************
 * @file    RobotiqHandPlugin.cpp
 * @brief   Plugin to control Robotiq S Model hand
 * @details Provides interface betwen Robotiq ROS messages and gazebo
 ********************************************************************************************************
 */

/*
 * Based on IRobot Hand plugin by OSRF
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


/*** INCLUDE FILES ***/
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "drcsim_gazebo_ros_plugins//RobotiqHandPlugin.h"
#include "robotiq_s_model_control/SModel_robot_output.h"
#include "robotiq_s_model_control/SModel_robot_input.h"
#include <ros/ros.h>

////////////////////////////////////////////////////////////////////////////////
// Based on the IRobotHand Plugin
RobotiqHandPlugin::RobotiqHandPlugin()
{
    printf("RobotiqHandPlugin constructor\n");
    for (int i = 0; i < 5; ++i)
    {
	this->kp_position[i] = 1.0;
	this->ki_position[i]  = 0.0;
	this->kd_position[i]  = 0.0;
	this->i_position_effort_min[i] = 0.0;
	this->i_position_effort_max[i] = 0.0;
	this->kp_velocity[i]  = 0.1;
	this->ki_velocity[i]  = 0.0;
	this->kd_velocity[i]  = 0.0;
	this->i_velocity_effort_min[i] = 0.0;
	this->i_velocity_effort_max[i] = 0.0;
    }

    this->errorTerms.resize(5);  // hand has 5 DOF
    for (unsigned i = 0; i < 5; ++i)
    {
	this->errorTerms[i].q_p = 0;
	this->errorTerms[i].d_q_p_dt = 0;
	this->errorTerms[i].q_i = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
RobotiqHandPlugin::~RobotiqHandPlugin()
{
    gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->rosNode->shutdown();
    this->rosQueue.clear();
    this->rosQueue.disable();
    this->callbackQueeuThread.join();
    delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->sdf = _sdf;

    if(!this->sdf->HasElement("side") ||
       !this->sdf->GetElement("side")->GetValue()->Get(this->side) ||
       ((this->side != "l_") && (this->side != "r_")))
    {
	gzerr << "Failed to determine which hand we're controlling; "
	    "aborting plugin load." << std::endl;
	return;
    }

    gzlog << "RobotiqHandPlugin loading for " << this->side << " hand." << std::endl;

    if(!this->FindJoints())
    {
	return;
    }

    //this->SetJointSpringDamper();

    // save thumb upper limit
    //this->thumbUpperLimit = this->fingerBaseJoints[2]->GetUpperLimit(0).Radian();
    //this->thumbAntagonistAngle = 0.0;

    // Load ROS
    // initialize ros
    if (!ros::isInitialized())
    {
	gzerr << "Not loading plugin since ROS hasn't been "
	      << "properly initialized.  Try starting gazebo with ros plugin:\n"
	      << "  gazebo -s libgazebo_ros_api_plugin.so\n";
	return;
    }

    // ros stuff
    this->rosNode = new ros::NodeHandle("");

    // publish multi queue
    this->pmq.startServiceThread();

    std::string sensorStr =  "/left_hand/SModelRobotInput";
    std::string controlStr = "/left_hand/SModelRobotOutput";
    if (this->side != "l_") {
	sensorStr =  "/right_hand/SModelRobotInput";
	controlStr = "/right_hand/SModelRobotOutput";
    }

    // broadcasts state
    this->pubHandleStateQueue = this->pmq.addPub<robotiq_s_model_control::SModel_robot_input>();
    this->pubHandleState = this->rosNode->advertise<robotiq_s_model_control::SModel_robot_input>(sensorStr, 100, true);

    // subscribe to user published handle control commands
    ros::SubscribeOptions handleCommandSo =
	ros::SubscribeOptions::create<robotiq_s_model_control::SModel_robot_output>(
	    controlStr, 100,
	    boost::bind(&RobotiqHandPlugin::SetHandleCommand, this, _1),
	    ros::VoidPtr(), &this->rosQueue);
    // Enable TCP_NODELAY since TCP causes bursty communication with high jitter,
    handleCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
    this->subHandleCommand = this->rosNode->subscribe(handleCommandSo);

    // controller time control
    this->lastControllerUpdateTime = this->world->GetSimTime();

    // start callback queue
    this->callbackQueeuThread = boost::thread(boost::bind(&RobotiqHandPlugin::RosQueueThread, this));

    // connect to gazebo world update
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RobotiqHandPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::SetHandleCommand(const robotiq_s_model_control::SModel_robot_output::ConstPtr &_msg)
{
    boost::mutex::scoped_lock lock(this->controlMutex);
    // Just looking at rPRA, rPRB, rPRC as three position values

    // Update handleCommand
    this->handleCommand.rPRA = _msg->rPRA;
    this->handleCommand.rPRB = _msg->rPRB;
    this->handleCommand.rPRC = _msg->rPRC;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::UpdateStates()
{
    gazebo::common::Time curTime = this->world->GetSimTime();

    if (curTime > this->lastControllerUpdateTime)
    {
	// gather robot state data and publish them
	this->GetAndPublishHandleState(curTime);
	this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());
	this->lastControllerUpdateTime = curTime;
    }
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::GetAndPublishHandleState(const gazebo::common::Time &_curTime)
{
    boost::mutex::scoped_lock lock(this->controlMutex);

    //this->handleState.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

    this->handleState.gACT = 0;
    this->handleState.gMOD = 0;
    this->handleState.gGTO = 0;
    this->handleState.gIMC = 0;
    this->handleState.gSTA = 0;
    this->handleState.gDTA = 0; // finger A (2 = has stopped due to a contact while closing)
    this->handleState.gDTB = 0; // Finger B (3 = at requested position)
    this->handleState.gDTC = 0; // Finger C (0,1 = ?)
    this->handleState.gDTS = 0; // Scissor
    this->handleState.gFLT = 0; // 0 is no fault
    this->handleState.gPRA = this->handleCommand.rPRA; // echo of requested position for finger a
    this->handleState.gPOA = 0; // current position of finger A
    this->handleState.gCUA = 0; // current of finger A
    this->handleState.gPRB = this->handleCommand.rPRB;
    this->handleState.gPOB = 0;
    this->handleState.gCUB = 0;
    this->handleState.gPRC = this->handleCommand.rPRC;
    this->handleState.gPOC = 0;
    this->handleState.gCUC = 0;
    this->handleState.gPRS = 0;
    this->handleState.gPOS = 0;
    this->handleState.gCUS = 0;

    // publish robot states
    this->pubHandleStateQueue->push(this->handleState, this->pubHandleState);

    //tmpCnt++;

    /*
    int ival = this->handleCommand.rPRA;
    int ival2 = this->handleCommand.rPRB;
    int ival3 = this->handleCommand.rPRC;
    if ((tmpCnt % 1000) == 0) {
	std::cout << this->side << " rPRA=" << ival << " rPRB=" << ival2 << " rPRC=" << ival3 << std::endl;
    }
    */
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::UpdatePIDControl(double _dt)
{
    boost::mutex::scoped_lock lock(this->controlMutex);


    for (int j = 0; j < 3; j++)
    {
	double torque;
	double target;
	if (j == 0) {
	    target = this->handleCommand.rPRA * 0.006;
	}
	else if (j == 1) {
	    target = this->handleCommand.rPRB * 0.006;
	}
	else if (j == 2) {
	    target = this->handleCommand.rPRC * 0.006;
	}


	double current;

	double baseJointPos = 0;
	double baseJointVel = 0;
	double flexureFlexJointPos = 0;
	double flexureFlexJointVel = 0;
	double currentPos = 0;
	double currentVel = 0;

	baseJointPos = this->fingerBaseJoints[j]->GetAngle(0).Radian();
	baseJointVel = this->fingerBaseJoints[j]->GetVelocity(0);
	currentPos = baseJointPos;
	currentVel = baseJointVel;

	double kp, ki, kd, i_effort_max, i_effort_min;

	// set state  for position control
	current = currentPos;

	kp = this->kp_position[j];
	ki = this->ki_position[j];
	kd = this->kd_position[j];
	i_effort_min = this->i_position_effort_min[j];
	i_effort_max = this->i_position_effort_max[j];

	double q_p = target - current;

	this->errorTerms[j].q_p = q_p;
	if (!gazebo::math::equal(_dt, 0.0)) {
	    this->errorTerms[j].d_q_p_dt = (q_p - this->errorTerms[j].q_p) / _dt;
	}
	this->errorTerms[j].q_i = gazebo::math::clamp(
	    this->errorTerms[j].q_i + _dt * this->errorTerms[j].q_p,
	    i_effort_min, i_effort_max);

	// use gain params to compute force cmd
	torque = kp * this->errorTerms[j].q_p +
	    ki * this->errorTerms[j].q_i +
	    kd * this->errorTerms[j].d_q_p_dt;

	//torque = -1.0;
	this->fingerBaseJoints[j]->SetForce(0, torque);
    }
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::GetAndPushBackJoint(const std::string& _joint_name,
					    gazebo::physics::Joint_V& _joints)
{
    gazebo::physics::JointPtr joint = this->model->GetJoint(_joint_name);

    /*
    int jntcnt = this->model->GetJointCount();
    gazebo::physics::Joint_V jnts = this->model->GetJoints();
    std::string vname;
    for (unsigned int i = 0; i < jntcnt; i++) {
	vname = jnts[i]->GetName();
	printf("jntlist %d %s\n", i, vname.c_str());
    }
    */

    if(!joint)
    {
	gzerr << "Failed to find joint: " << _joint_name <<
	    "; aborting plugin load." << std::endl;
	return false;
    }
    _joints.push_back(joint);
    gzlog << "RobotiqHandPlugin found joint: " << _joint_name << std::endl;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::KpKdToCFMERP(const double _dt,
				     const double _kp, const double _kd,
				     double &_cfm, double &_erp)
{
    /// \TODO: check for NaN cases
    _erp = _dt * _kp / ( _dt * _kp + _kd );
    _cfm = 1.0 / ( _dt * _kp + _kd );
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::CFMERPToKpKd(const double _dt,
				     const double _cfm, const double _erp,
				     double &_kp, double &_kd)
{
    /// \TODO: check for NaN cases
    _kp = _erp / (_dt * _cfm);
    _kd = (1.0 - _erp) / _cfm;
}

////////////////////////////////////////////////////////////////////////////////
double RobotiqHandPlugin::HandleControlFlexValueToFlexJointAngle(int _value)
{
    // convert from int32 to joint angle in radians

    // _value is taken as motor rotation in radians.
    // in practice, spool diameter varies from 10mm to 14mm.
    const double intToMotorAngle = 1.0;
    double motorAngle = intToMotorAngle * static_cast<double>(_value);
    const double spoolDiameter = 0.012;
    double tendonLength = spoolDiameter * motorAngle;
    const double tendonLengthToAngle = 0.01;  // wild guess
    double jointAngle = tendonLengthToAngle * tendonLength;
    return jointAngle;
}

////////////////////////////////////////////////////////////////////////////////
double RobotiqHandPlugin::HandleControlSpreadValueToSpreadJointAngle(int _value)
{
    // convert from int32 to joint angle in radians
    /// \TODO: figure out what is the unit of incoming integer.
    const double intToMotorAngle = 1.0;
    double motorAngle = intToMotorAngle * static_cast<double>(_value);
    const double reductionRatio = 1.0/1000.0;
    return reductionRatio * motorAngle;
}

////////////////////////////////////////////////////////////////////////////////
bool RobotiqHandPlugin::FindJoints()
{
    this->flexureFlexJoints.resize(this->numFingers);

    // Load up the joints we expect to use, finger by finger.
    gazebo::physics::JointPtr joint;
    if (this->side == "l_")
    {
	if(!this->GetAndPushBackJoint("l_finger_1_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
	if(!this->GetAndPushBackJoint("l_finger_2_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
	if(!this->GetAndPushBackJoint("l_finger_middle_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
    }
    else {
	if(!this->GetAndPushBackJoint("r_finger_1_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
	if(!this->GetAndPushBackJoint("r_finger_2_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
	if(!this->GetAndPushBackJoint("r_finger_middle_joint_1", this->fingerBaseJoints)) {
	    return false;
	}
    }

    gzlog << "RobotiqHandPlugin found all joints for " << this->side << " hand." << std::endl;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void RobotiqHandPlugin::SetJointSpringDamper()
{
    return;
    // Fake springiness by setting joint limits to 0 and modifying cfm/erp.
    // TODO: implement a generic spring in Gazebo that will work with any
    // physics engine.

    // (this->numFlexLinks + 2) flex joints @ 0.029 in-lbs/deg per
    // iRobot estimates
    const double flexJointKp = 0.187733 * (this->numFlexLinks + 2);
    const double flexJointKd = 0.01;  // wild guess
    const double twistJointKp =
	0.187733 * (this->numFlexLinks + 2) * 2.0;  // wild guess
    const double twistJointKd = 0.01;  // wild guess
    // 0.0031 in-lbs / deg per iRobot estimates
    const double baseJointKp = 0.020068;
    const double baseJointKd = 0.1;  // wild guess
    const double baseRotationJointKp = 0.0;  // no spring there
    const double baseRotationJointKd = 1.0;  // wild guess

    // 0.23 in-lbs preload per iRobot data
    const double baseJointPreloadTorque = 0.0259865;
    // calculate position for preloaded tension
    const double baseJointPreloadJointPosition =
	baseJointPreloadTorque / baseJointKp;

    // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
    //   flexJointKp, flexJointKd, this->flexJointCFM, this->flexJointERP);
    // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
    //   twistJointKp, twistJointKd, this->twistJointCFM, this->twistJointERP);
    // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
    //   baseJointKp, baseJointKd, this->baseJointCFM, this->baseJointERP);

    // Handle the flex/twist joints in the flexible section
    for(std::vector<gazebo::physics::Joint_V>::iterator it =
	    this->flexureFlexJoints.begin();
	it != this->flexureFlexJoints.end();
	++it)
    {
	for(gazebo::physics::Joint_V::iterator iit = it->begin();
	    iit != it->end();
	    ++iit)
	{
	    // Assume that the joints are ordered flex then twist, in pairs.
	    //(*iit)->SetStiffnessDamping(0, flexJointKp, flexJointKd);
	    // (*iit)->SetAttribute("lo_stop", 0, 0.0);
	    // (*iit)->SetAttribute("hi_stop", 0, 0.0);
	    // (*iit)->SetAttribute("stop_cfm", 0, this->flexJointCFM);
	    // (*iit)->SetAttribute("stop_erp", 0, this->flexJointERP);
	}
    }

    for(std::vector<gazebo::physics::Joint_V>::iterator it =
	    this->flexureTwistJoints.begin();
	it != this->flexureTwistJoints.end();
	++it)
    {
	for(gazebo::physics::Joint_V::iterator iit = it->begin();
	    iit != it->end();
	    ++iit)
	{
	    ///(*iit)->SetStiffnessDamping(0, twistJointKp, twistJointKd);
	    // (*iit)->SetAttribute("lo_stop", 0, 0.0);
	    // (*iit)->SetAttribute("hi_stop", 0, 0.0);
	    // (*iit)->SetAttribute("stop_cfm", 0, this->twistJointCFM);
	    // (*iit)->SetAttribute("stop_erp", 0, this->twistJointERP);
	}
    }

    // Handle the base joints, which are spring-loaded.
    for(gazebo::physics::Joint_V::iterator it = this->fingerBaseJoints.begin();
	it != this->fingerBaseJoints.end();
	++it)
    {
	//(*it)->SetStiffnessDamping(0, baseJointKp, baseJointKd,
	//-baseJointPreloadJointPosition);
	// (*it)->SetAttribute("lo_stop", 0, -baseJointPreloadJointPosition);
	// (*it)->SetAttribute("hi_stop", 0, -baseJointPreloadJointPosition);
	// (*it)->SetAttribute("stop_cfm", 0, this->baseJointCFM);
	// (*it)->SetAttribute("stop_erp", 0, this->baseJointERP);
    }

    // Handle the base rotation joints, which are not spring-loaded.
    for(gazebo::physics::Joint_V::iterator
	    it = this->fingerBaseRotationJoints.begin();
	it != this->fingerBaseRotationJoints.end();
	++it)
    {
	//(*it)->SetStiffnessDamping(0, baseRotationJointKp, baseRotationJointKd);
    }
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
