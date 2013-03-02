
#ifndef __AtlasSimInterfaceTypes_H
#define __AtlasSimInterfaceTypes_H

#include "AtlasSimVectorTypes.h"

#if BDI_OS_TYPE_win32

typedef unsigned __int64 uint64_t;

#else

#include <stdint.h>
#include <sys/types.h>

#endif

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

//#ifndef NOT_IN_DOXYGEN
//namespace Atlas {
//#endif

#define ATLAS_SIM_SOFTWARE_VERSION_STRING "1.0.3"
#define ATLAS_SIM_SOFTWARE_VERSION_MAJOR 1
#define ATLAS_SIM_SOFTWARE_VERSION_MINOR 0
#define ATLAS_SIM_SOFTWARE_VERSION_POINT 3


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Link and Joint Enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasLinkId
//!
//!  \brief   Enumerations identifying robot links.
//!
namespace Atlas {

typedef enum
{
	LINK_UNKNOWN      = -1,
	LINK_PELVIS       = 0,
	LINK_LTORSO       = 1,
	LINK_MTORSO       = 2,
	LINK_UTORSO       = 3,
	LINK_HEAD         = 4,
	LINK_L_UGLUT      = 5, 
	LINK_L_LGLUT      = 6, 
	LINK_L_ULEG       = 7, 
	LINK_L_LLEG       = 8,
	LINK_L_TALUS      = 9, 
	LINK_L_FOOT       = 10,
	LINK_R_UGLUT      = 11, 
	LINK_R_LGLUT      = 12, 
	LINK_R_ULEG       = 13, 
	LINK_R_LLEG       = 14,
	LINK_R_TALUS      = 15, 
	LINK_R_FOOT       = 16,
	LINK_L_CLAV       = 17,
	LINK_L_SCAP       = 18,
	LINK_L_UARM       = 19,
	LINK_L_LARM       = 20,
	LINK_L_FARM       = 21,
	LINK_L_HAND       = 22,
	LINK_R_CLAV       = 23,
	LINK_R_SCAP       = 24,
	LINK_R_UARM       = 25,
	LINK_R_LARM       = 26,
	LINK_R_FARM       = 27,
	LINK_R_HAND       = 28,
	NUM_LINKS

} AtlasLinkId;

}

///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasJointId
//!
//!  \brief   Enumerations identifying robot joints.
//!
namespace Atlas {

typedef enum
{
	JOINT_UNKNOWN     = -1,
	JOINT_BACK_LBZ    = 0,
	JOINT_BACK_MBY    = 1,
	JOINT_BACK_UBX    = 2,
	JOINT_NECK_AY     = 3,
	JOINT_L_LEG_UHZ   = 4,
	JOINT_L_LEG_MHX   = 5,
	JOINT_L_LEG_LHY   = 6,
	JOINT_L_LEG_KNY   = 7,
	JOINT_L_LEG_UAY   = 8,
	JOINT_L_LEG_LAX   = 9,
	JOINT_R_LEG_UHZ   = 10,
	JOINT_R_LEG_MHX   = 11,
	JOINT_R_LEG_LHY   = 12,
	JOINT_R_LEG_KNY   = 13,
	JOINT_R_LEG_UAY   = 14,
	JOINT_R_LEG_LAX   = 15,
	JOINT_L_ARM_USY   = 16,
	JOINT_L_ARM_SHX   = 17,
	JOINT_L_ARM_ELY   = 18,
	JOINT_L_ARM_ELX   = 19,
	JOINT_L_ARM_UWY   = 20,
	JOINT_L_ARM_MWX   = 21,
	JOINT_R_ARM_USY   = 22,
	JOINT_R_ARM_SHX   = 23,
	JOINT_R_ARM_ELY   = 24,
	JOINT_R_ARM_ELX   = 25,
	JOINT_R_ARM_UWY   = 26,
	JOINT_R_ARM_MWX   = 27,
	NUM_JOINTS

} AtlasJointId;

}


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
	AtlasJointDesired() {q_d = qd_d = f_d = 0.0f;}
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
	AtlasJointControlParams()
	{
		k_q_p = k_q_i = k_qd_p = 0.0f;
	}
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasControlDataToRobot
//!
//!  \brief   Structure for control output data to be streamed to robot sim.
//!
struct AtlasControlDataToRobot
{
	uint64_t timestamp;

	AtlasJointDesired       j[Atlas::NUM_JOINTS];
	AtlasJointControlParams jparams[Atlas::NUM_JOINTS];

	//! \brief  Default contructor.  All data members set to 0.
	AtlasControlDataToRobot() {timestamp = 0l;}
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
	AtlasJointState() {q = qd = f = 0.0f;}
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasIMUData
//!
//!  \brief   Structure for returned IMU readings.
//!
//!    All angular velocities and linear accelerations are expressed
//!    relative to an intertial frame, with respect to the IMU frame.
//!
//!    The orientation estimate is expressed relative to an intertial frame,
//!    with respect to the IMU frame.  Note: the z axis of the inertial frame
//!    is aligned with the gravity vector but is opposite in sign.
//!
//!
//!  NOTE: This could probably be replaced with ROS/Gazebo IMU sensor.
//!
struct AtlasIMUData
{
	uint64_t imu_timestamp;  //!<  Timestamp for IMU data [s*(10^-6)]

	AtlasQuaternion orientation_estimate; //!<  Approximate orientation, as quaternion
	AtlasVec3f      angular_velocity;     //!<  Angular velocity about x, y, z axes, in rad/s.
	AtlasVec3f      linear_acceleration;  //!<  Linear acceleration along x, y, z axes, in m/s^2.

	//! \brief  Default contructor.  All data members set to 0 or identity.
	AtlasIMUData() :
		imu_timestamp(0l),
		angular_velocity(0.0f),
		linear_acceleration(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasFootSensor
//!
//!  \brief   Structure for returned foot sensors.
//!
//!    This structure holds the force and moments exterted on a foot by
//!    the environment, in the foot's coordinate frame.
//!
struct AtlasFootSensor
{
	float fz;     //!<  Force exerted on foot along z axis, in N.
	float mx;     //!<  Moment exerted on foot about x axis, in N*m.
	float my;     //!<  Moment exerted on foot about y axis, in N*m.

	//! \brief  Default contructor.  All data members set to 0.
	AtlasFootSensor() {fz = mx = my = 0.0f;}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasFootSensorId
//!
//!  \brief   Enumerations identifying robot foot sensors.
//!
namespace Atlas {

enum
{
	FS_LEFT  = 0,
	FS_RIGHT,
	NUM_FOOT_SENSORS
};

}

///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasWristSensor
//!
//!  \brief   Structure for returned wrist sensors.
//!
//!    This structure holds the forces and moments exterted on a hand by
//!    the environment, in the hand's coordinate frame.
//!
struct AtlasWristSensor
{
	AtlasVec3f f;               //!<  Forces exerted on hand along x, y, z axes, in N
	AtlasVec3f m;               //!<  Moments exerted on hand about x, y, z axes, in N*m

	//! \brief  Default contructor.  All data members set to 0.
	AtlasWristSensor() :
		f(0.0f),
		m(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasWristSensorId
//!
//!  \brief   Enumerations identifying robot foot sensors.
//!
namespace Atlas {

enum
{
	WS_LEFT = 0,
	WS_RIGHT,
	NUM_WRIST_SENSORS
};

}

///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasControlDataFromRobot
//!
//!  \brief   Structure for current sim state to be streamed to robot sim control.
//!
struct AtlasControlDataFromRobot
{
	uint64_t seq_id;        //!<  (ben)
	
	double t;               //!< Robot/sim time

	AtlasJointState j[Atlas::NUM_JOINTS];

	AtlasIMUData     imu;                                     //!<  Packet of IMU data
	AtlasFootSensor  foot_sensors[Atlas::NUM_FOOT_SENSORS];   //!<  State of foot sensors
	AtlasWristSensor wrist_sensors[Atlas::NUM_WRIST_SENSORS]; //!<  State of wrist sensors

	AtlasVec3f pelvis_position;
	AtlasVec3f pelvis_velocity;

	AtlasControlDataFromRobot() :
		seq_id(0l),
		t(0.0),
		pelvis_position(0.0f),
		pelvis_velocity(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasBehaviorParameter
//!
//!  \brief   Enumeration identifying various parameters that can be set.
//!
namespace Atlas {

typedef enum
{
	BEHAVIOR_PARAM_DESIRED_YAW_RATE = 0,   //!< 1 value: yaw rate about z, rad/s
	BEHAVIOR_PARAM_DESIRED_VELOCITY,       //!< 2 values, x & y, m/s
	NUM_BEHAVIOR_PARAMS

} AtlasBehaviorParameterId;

}

///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasErrorCode
//!
//!  \brief   Enumeration identifying error codes that can be returned from function calls.
//!
namespace Atlas {

typedef enum
{
	NO_ERROR                         =  0,   //!< no error detected
	ERROR_UNSPECIFIED                = -1,   //!< unspecified error
	ERROR_VALUE_OUT_OF_RANGE         = -2,   //!< passed value is out of range
	ERROR_INVALID_INDEX              = -3,   //!< passed index is invalid (too low or too high)
	ERROR_FAILED_TO_START_BEHAVIOR   = -4,   //!< robot failed to start desired behavior
	ERROR_NO_ACTIVE_BEHAVIOR         = -5,   //!< robot has no active behavior
	ERROR_NO_SUCH_BEHAVIOR           = -6,   //!< behavior doesn't exist
	ERROR_BEHAVIOR_NOT_IMPLEMENTED   = -7,   //!< behavior exists but not implemented
	ERROR_TIME_RAN_BACKWARD          = -8,   //!< a time earlier than previous times was given
	ERROR_INVALID_PARAMETER          = -9,   //!< passed parameter is not valid in the current behavior
	NUM_ERRORS

} AtlasErrorCode;

}

//#ifndef NOT_IN_DOXYGEN
//} // end namespace Atlas
//#endif

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasSimInterfaceTypes_H

