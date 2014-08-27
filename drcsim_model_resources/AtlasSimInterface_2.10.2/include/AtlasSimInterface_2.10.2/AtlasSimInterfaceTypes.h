
#ifndef __AtlasSimInterfaceTypes_H
#define __AtlasSimInterfaceTypes_H

#include "AtlasControlTypes.h"
#include "AtlasVectorTypes.h"

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

/* Name mangled so that perl scan from Makefile doesn't find these 
#define ATLAS_SIM_MANGLED_SOFTWARE_VERSION_STRING "1.1.1"
#define ATLAS_SIM_MANGLED_SOFTWARE_VERSION_MAJOR 1
#define ATLAS_SIM_MANGLED_SOFTWARE_VERSION_MINOR 1
#define ATLAS_SIM_MANGLED_SOFTWARE_VERSION_POINT 1
*/
/* This should periodically be copied from ../AtlasRobotInterface/AtlasInterfaceTypes.h */
#define ATLAS_SIM_SOFTWARE_VERSION_STRING "2.10.2"
#define ATLAS_SIM_SOFTWARE_VERSION_MAJOR 2
#define ATLAS_SIM_SOFTWARE_VERSION_MINOR 10
#define ATLAS_SIM_SOFTWARE_VERSION_POINT 2

using namespace Atlas;

#ifndef NOT_IN_DOXYGEN
namespace AtlasSim {
#endif


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Robot state types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointState
//!
//!  \brief   Structure for holding robot joint state.
//!
struct AtlasJointState
{
	float q;       //!<  Measured orientation or position of joint.
	float qd;      //!<  Measured velocity of joint.
	float f;       //!<  Measured torque of joint.

	//! \brief  Default contructor.  All data members set to 0.
	AtlasJointState() :
		q(0.0f),
		qd(0.0f),
		f(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasRobotState
//!
//!  \brief   Structure for current sim state to be streamed to robot sim control.
//!
struct AtlasRobotState
{
	double t;               //!< Sim time

	AtlasJointState j[Atlas::NUM_JOINTS];

	AtlasIMUData     imu;                                     //!<  Packet of IMU data
	AtlasFootSensor  foot_sensors[Atlas::NUM_FOOT_SENSORS];   //!<  State of foot sensors
	AtlasWristSensor wrist_sensors[Atlas::NUM_WRIST_SENSORS]; //!<  State of wrist sensors

	AtlasRobotState() :
		t(0.0)
		{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointDesired
//!
//!  \brief   Structure for specifying joint setpoints
//!
struct AtlasJointDesired
{
	float q_d;            //!< Desired position of joint.
	float qd_d;           //!< Desired velocity of joint.
	float f_d;            //!< Desired torque of joint.

	//! \brief  Default contructor.  All data members set to 0.
	AtlasJointDesired() :
		q_d(0.0f),
		qd_d(0.0f),
		f_d(0.0f)
	{}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Control input types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointControlParams
//!
//!  \brief   Structure for specifying controller parameters
//!
//!    This structure contains the gains to be applied to a joint.
//!
//!    The final joint command will be:
//!
//!        k_q_p     * ( q_d - q )       +
//!        k_q_i     * 1/s * ( q_d - q ) +
//!        k_qd_p    * ( qd_d - qd )     +
//!        f_d
//!
//!    The f_d term is from AtlasJointDesired.
//!
struct AtlasJointControlParams
{
	float k_q_p;      //!<  Position error gain, in N*m/rad.
	float k_q_i;      //!<  Integral of position error gain, in N*m/(rad*s).
	float k_qd_p;     //!<  Derivative error gain, in N*m/(rad/s).

	//! \brief  Default constructor.  All data members set to 0.
	AtlasJointControlParams() :
		k_q_p(0.0f),
		k_q_i(0.0f),
		k_qd_p(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasControlInput
//!
//!  \brief   Structure for current sim state to be streamed to robot sim control.
//!
struct AtlasControlInput
{
	AtlasJointDesired j[Atlas::NUM_JOINTS];             //!<  Setpoints for joint joints.
	AtlasJointControlParams jparams[Atlas::NUM_JOINTS]; //!<  Gains for joint joints.

	AtlasBehaviorStandParams      stand_params;         //!<  Control parameters for Stand.
	AtlasBehaviorStepParams       step_params;          //!<  Control parameters for Step.
	AtlasBehaviorWalkParams       walk_params;          //!<  Control parameters for Walk.
	AtlasBehaviorManipulateParams manipulate_params;    //!<  Control parameters for Manipulate.
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Control output types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasControlOutput
//!
//!  \brief   Structure for control data to be streamed to robot sim.
//!
//!    This structure is for holding data that is to be streamed to the robot
//!    at a high rate.  Most of this data will be parameters for actuator
//!    control.
//!
struct AtlasControlOutput
{
	//!
	//!  \brief  Joint torques to be applied to simulation.
	//!
	double f_out[Atlas::NUM_JOINTS];

	//!
	//!  \brief  World position estimate for reference when giving desired step positions.
	//!
	AtlasPositionData pos_est;

	//!
	//!  \brief  World position estimate for feet.
	//!
	AtlasVec3f foot_pos_est[Atlas::NUM_FEET];

	AtlasBehaviorFeedback           behavior_feedback;        //!< General behavior feedback.
	AtlasBehaviorStandFeedback      stand_feedback;           //!< Feedback specific to the Stand behavior.
	AtlasBehaviorStepFeedback       step_feedback;            //!< Feedback specific to the Step behavior.
	AtlasBehaviorWalkFeedback       walk_feedback;            //!< Feedback specific to the Walk behavior.
	AtlasBehaviorManipulateFeedback manipulate_feedback;      //!< Feedback specific to the Manipulate behavior.

	//!
	//!  \brief  Default constructor.  All data members set to 0.
	//!
	AtlasControlOutput()
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasErrorCode
//!
//!  \brief   Enumeration identifying error codes that can be returned from function calls.
//!
#ifdef ERROR_INVALID_INDEX
#undef ERROR_INVALID_INDEX
#endif

typedef enum
{
	NO_ERRORS                        =  0,   //!< no error detected
	ERROR_UNSPECIFIED                = -1,   //!< unspecified error
	ERROR_VALUE_OUT_OF_RANGE         = -2,   //!< passed value is out of range
	ERROR_INVALID_INDEX              = -3,   //!< passed index is invalid (too low or too high)
	ERROR_FAILED_TO_START_BEHAVIOR   = -4,   //!< robot failed to start desired behavior
	ERROR_NO_ACTIVE_BEHAVIOR         = -5,   //!< robot has no active behavior
	ERROR_NO_SUCH_BEHAVIOR           = -6,   //!< behavior doesn't exist
	ERROR_BEHAVIOR_NOT_IMPLEMENTED   = -7,   //!< behavior exists but not implemented
	ERROR_TIME_RAN_BACKWARD          = -8,   //!< a time earlier than previous times was given
	ERROR_INVALID_RESULTS            = -9,   //!< returned results are invalid
	NUM_ERROR_CODES

} AtlasErrorCode;

///@}

#ifndef NOT_IN_DOXYGEN
} // end namespace AtlasSim
#endif

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasSimInterfaceTypes_H

