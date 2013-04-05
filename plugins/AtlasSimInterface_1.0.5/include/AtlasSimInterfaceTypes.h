
#ifndef __AtlasSimInterfaceTypes_H
#define __AtlasSimInterfaceTypes_H

#include "AtlasControlTypes.h"
#include "AtlasVectorTypes.h"

//#if BDI_OS_TYPE_win32
//
//typedef unsigned __int64 uint64_t;
//
//#else
//
//#include <stdint.h>
//#include <sys/types.h>
//
//#endif

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

#define ATLAS_SIM_SOFTWARE_VERSION_STRING "1.0.5"
#define ATLAS_SIM_SOFTWARE_VERSION_MAJOR 1
#define ATLAS_SIM_SOFTWARE_VERSION_MINOR 0
#define ATLAS_SIM_SOFTWARE_VERSION_POINT 5

using namespace Atlas;

#ifndef NOT_IN_DOXYGEN
namespace AtlasSim {
#endif


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name To-robot types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasJointDesired
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


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasJointControlParams
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
//!  \struct    AtlasControlDataToRobot
//!
//!  \brief   Structure for control output data to be streamed to robot sim.
//!
//!    This structure is for holding data that is to be streamed to the robot
//!    at a high rate.  Most of this data will be parameters for actuator
//!    control.
//!
struct AtlasControlDataToRobot
{
	/*
	 *  Values can be set using either array notation or by using a macro
	 *   define above.  For example, the following are equivalent:
	 *
	 *   in.j[JOINT_BACK_LBZ].q_d = 1.0f;
	 *   in.back_lbz.q_d = 1.0f;
	 *
	 *   and also equivalent:
	 *
	 *   in.jparams[JOINT_BACK_LBZ].k_q_p = 0.0f;
	 *   in.back_lbz_params.k_q_p = 0.0f;
	 */
	AtlasJointDesired j[Atlas::NUM_JOINTS];
	AtlasJointControlParams jparams[Atlas::NUM_JOINTS];

	AtlasPositionData pos_est;                  //!< World position estimate for reference when giving desired step positions
	AtlasVec3f foot_pos_est[Atlas::NUM_FEET];   //!< World position estimate for feet

	uint32_t	current_step_index;        //!<  Current step index in multi_step or single_step modes.

	//! \brief  Default constructor.  All data members set to 0.
	AtlasControlDataToRobot() : current_step_index(0)
	{}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name From-robot types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasJointState
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
//!  \struct    AtlasControlDataFromRobot
//!
//!  \brief   Structure for current sim state to be streamed to robot sim control.
//!
struct AtlasControlDataFromRobot
{
	double t;               //!< Sim time

	AtlasJointState j[Atlas::NUM_JOINTS];

	AtlasIMUData     imu;                                     //!<  Packet of IMU data
	AtlasFootSensor  foot_sensors[Atlas::NUM_FOOT_SENSORS];   //!<  State of foot sensors
	AtlasWristSensor wrist_sensors[Atlas::NUM_WRIST_SENSORS]; //!<  State of wrist sensors

	/*
	 *  Behavior params.
	 */
	AtlasBehaviorStandParams          stand_params;
	AtlasBehaviorSingleStepWalkParams singlestep_walk_params;
	AtlasBehaviorMultiStepWalkParams  multistep_walk_params;

	AtlasControlDataFromRobot() :
		t(0.0)
	{}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Misc enumerations.
//!
///@{

///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasErrorCode
//!
//!  \brief   Enumeration identifying error codes that can be returned from function calls.
//!
///namespace Atlas {

#ifdef ERROR_INVALID_INDEX
#undef ERROR_INVALID_INDEX
#endif

typedef enum
{
	NO_ERRORS                         =  0,   //!< no error detected
	ERROR_UNSPECIFIED                = -1,   //!< unspecified error
	ERROR_VALUE_OUT_OF_RANGE         = -2,   //!< passed value is out of range
	ERROR_INVALID_INDEX              = -3,   //!< passed index is invalid (too low or too high)
	ERROR_FAILED_TO_START_BEHAVIOR   = -4,   //!< robot failed to start desired behavior
	ERROR_NO_ACTIVE_BEHAVIOR         = -5,   //!< robot has no active behavior
	ERROR_NO_SUCH_BEHAVIOR           = -6,   //!< behavior doesn't exist
	ERROR_BEHAVIOR_NOT_IMPLEMENTED   = -7,   //!< behavior exists but not implemented
	ERROR_TIME_RAN_BACKWARD          = -8,   //!< a time earlier than previous times was given
	ERROR_INVALID_NEXT_STEP_INDEX    = -9,   //!< invalid step index was given for next step
	ERROR_INVALID_NEXT_STEP_FOOT_POSITION = -10,  //!< invalid foot position was given for next step
	NUM_ERROR_CODES

} AtlasErrorCode;

///}

///@}

#ifndef NOT_IN_DOXYGEN
} // end namespace AtlasSim
#endif

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasSimInterfaceTypes_H

