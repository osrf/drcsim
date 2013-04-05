
#ifndef __AtlasCommonTypes_H
#define __AtlasCommonTypes_H

#include "AtlasVectorTypes.h"

#if BDI_OS_TYPE_win32

//typedef unsigned __int64 uint64_t;
#include "bdiRTTypes.h"
#else

#include <stdint.h>
#include <sys/types.h>

#endif

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif


#ifndef NOT_IN_DOXYGEN
namespace Atlas {
#endif

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
///namespace Atlas {

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

///}

///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasJointId
//!
//!  \brief   Enumerations identifying robot joints.
//!
///namespace Atlas {

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

///}

//#define back_lbz  j[Atlas::JOINT_BACK_LBZ]
//#define back_mby  j[Atlas::JOINT_BACK_MBY]
//#define back_ubx  j[Atlas::JOINT_BACK_UBX]
//#define neck_ay   j[Atlas::JOINT_NECK_AY]
//#define l_leg_uhz j[Atlas::JOINT_L_LEG_UHZ]
//#define l_leg_mhx j[Atlas::JOINT_L_LEG_MHX]
//#define l_leg_lhy j[Atlas::JOINT_L_LEG_LHY]
//#define l_leg_kny j[Atlas::JOINT_L_LEG_KNY]
//#define l_leg_uay j[Atlas::JOINT_L_LEG_UAY]
//#define l_leg_lax j[Atlas::JOINT_L_LEG_LAX]
//#define r_leg_uhz j[Atlas::JOINT_R_LEG_UHZ]
//#define r_leg_mhx j[Atlas::JOINT_R_LEG_MHX]
//#define r_leg_lhy j[Atlas::JOINT_R_LEG_LHY]
//#define r_leg_kny j[Atlas::JOINT_R_LEG_KNY]
//#define r_leg_uay j[Atlas::JOINT_R_LEG_UAY]
//#define r_leg_lax j[Atlas::JOINT_R_LEG_LAX]
//#define l_arm_usy j[Atlas::JOINT_L_ARM_USY]
//#define l_arm_shx j[Atlas::JOINT_L_ARM_SHX]
//#define l_arm_ely j[Atlas::JOINT_L_ARM_ELY]
//#define l_arm_elx j[Atlas::JOINT_L_ARM_ELX]
//#define l_arm_uwy j[Atlas::JOINT_L_ARM_UWY]
//#define l_arm_mwx j[Atlas::JOINT_L_ARM_MWX]
//#define r_arm_usy j[Atlas::JOINT_R_ARM_USY]
//#define r_arm_shx j[Atlas::JOINT_R_ARM_SHX]
//#define r_arm_ely j[Atlas::JOINT_R_ARM_ELY]
//#define r_arm_elx j[Atlas::JOINT_R_ARM_ELX]
//#define r_arm_uwy j[Atlas::JOINT_R_ARM_UWY]
//#define r_arm_mwx j[Atlas::JOINT_R_ARM_MWX]
//
//#define back_lbz_params  jparams[Atlas::JOINT_BACK_LBZ]
//#define back_mby_params  jparams[Atlas::JOINT_BACK_MBY]
//#define back_ubx_params  jparams[Atlas::JOINT_BACK_UBX]
//#define neck_ay_params   jparams[Atlas::JOINT_NECK_AY]
//#define l_leg_uhz_params jparams[Atlas::JOINT_L_LEG_UHZ]
//#define l_leg_mhx_params jparams[Atlas::JOINT_L_LEG_MHX]
//#define l_leg_lhy_params jparams[Atlas::JOINT_L_LEG_LHY]
//#define l_leg_kny_params jparams[Atlas::JOINT_L_LEG_KNY]
//#define l_leg_uay_params jparams[Atlas::JOINT_L_LEG_UAY]
//#define l_leg_lax_params jparams[Atlas::JOINT_L_LEG_LAX]
//#define r_leg_uhz_params jparams[Atlas::JOINT_R_LEG_UHZ]
//#define r_leg_mhx_params jparams[Atlas::JOINT_R_LEG_MHX]
//#define r_leg_lhy_params jparams[Atlas::JOINT_R_LEG_LHY]
//#define r_leg_kny_params jparams[Atlas::JOINT_R_LEG_KNY]
//#define r_leg_uay_params jparams[Atlas::JOINT_R_LEG_UAY]
//#define r_leg_lax_params jparams[Atlas::JOINT_R_LEG_LAX]
//#define l_arm_usy_params jparams[Atlas::JOINT_L_ARM_USY]
//#define l_arm_shx_params jparams[Atlas::JOINT_L_ARM_SHX]
//#define l_arm_ely_params jparams[Atlas::JOINT_L_ARM_ELY]
//#define l_arm_elx_params jparams[Atlas::JOINT_L_ARM_ELX]
//#define l_arm_uwy_params jparams[Atlas::JOINT_L_ARM_UWY]
//#define l_arm_mwx_params jparams[Atlas::JOINT_L_ARM_MWX]
//#define r_arm_usy_params jparams[Atlas::JOINT_R_ARM_USY]
//#define r_arm_shx_params jparams[Atlas::JOINT_R_ARM_SHX]
//#define r_arm_ely_params jparams[Atlas::JOINT_R_ARM_ELY]
//#define r_arm_elx_params jparams[Atlas::JOINT_R_ARM_ELX]
//#define r_arm_uwy_params jparams[Atlas::JOINT_R_ARM_UWY]
//#define r_arm_mwx_params jparams[Atlas::JOINT_R_ARM_MWX]

///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasFootId
//!
//!  \brief   Enumerations identifying robot feet.
//!
///namespace Atlas {

typedef enum
{
	FOOT_LEFT         = 0,
	FOOT_RIGHT        = 1,
	NUM_FEET
} AtlasFootId;

///}


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Behavior control types.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasBehaviorStepParams
//!
//!  \brief   Structure for desired foot step data.
//!
struct AtlasBehaviorStepParams
{
	uint32_t step_index;	//!<  Step index, starting from 1, monotonically increasing during walking
							//!<  resets to 1 if robot leaves walk behaviors

	uint8_t foot_index;		//!<  Foot index (Left/Right) see AtlasFootId enum

	float duration;			//!<  Step duration
	AtlasVec3f position;	//!<   Foot position in Atlas world frame 
	float yaw;				//!<  Foot orientation yaw component
	

	///////////////////////////////////////////////////////////
	//! ** Currently Unused Parameters -- Subject to change **
	///////////////////////////////////////////////////////////
	AtlasVec3f normal;		//!<  Foot ground normal 
	float swing_height;		//!<  Step apex swing height as measured from the midpoint 
							//!<  between the feet.  


	AtlasBehaviorStepParams() :
		step_index(0l),
		foot_index(0),
		position(0.0f,0.0f,0.0f),
		yaw(0.0f),
		duration(0.7f),
		normal(0.0f,0.0f,1.0f)
	{}

	AtlasBehaviorStepParams(uint32_t _step_index,
		uint8_t _foot_index,
		float _duration,
		AtlasVec3f _position,
		float _yaw,
		AtlasVec3f _normal,
		float _swing_height)
		:
		step_index(_step_index),
		foot_index(_foot_index),
		duration(_duration),
		position(_position),
		yaw(_yaw),
		normal(_normal),
		swing_height(_swing_height)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasBehaviorStandParams
//!
//!  \brief   Structure for parameters for the Stand behavior.
//!
struct AtlasBehaviorStandParams
{
	bool  use_desired_pelvis_height;
	float desired_pelvis_height;
	float desired_pelvis_yaw;
	float desired_pelvis_lat;
	// etc.

	AtlasBehaviorStandParams() :
		use_desired_pelvis_height(false),
		desired_pelvis_height(0.8f),
		desired_pelvis_yaw(0.0f),
		desired_pelvis_lat(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasBehaviorSingleStepWalkParams
//!
//!  \brief   Structure for parameters for the SingleStep behavior.
//!
struct AtlasBehaviorSingleStepWalkParams
{
	AtlasBehaviorStepParams desired_step;
	// etc.
};


///////////////////////////////////////////////////////////
//!
//!  \struct    AtlasBehaviorMultiStepWalkParams
//!
//!  \brief   Structure for parameters for the MultiStep behavior.
//!
#define NUM_MULTISTEP_WALK_STEPS 4

struct AtlasBehaviorMultiStepWalkParams
{
	AtlasBehaviorStepParams step_data[NUM_MULTISTEP_WALK_STEPS];

	bool use_demo_walk;	// flag to use demo walk

	AtlasBehaviorMultiStepWalkParams() : use_demo_walk(true) {;}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name From-robot types and enumerations.
//!
///@{


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
//!  \struct    AtlasPositionData
//!
//!  \brief   Structure for returned Atlas robot state estimates
//!
//!    All positions and velocities expressed in the world frame as sensed by
//!    the IMU.
//!
struct AtlasPositionData
{
	AtlasVec3f      position;	//!<  Position estimate of robot pelvis (x,y,z) in meters
	AtlasVec3f      velocity;	//!<  Velocity estimate of robot pelvis (xd,yd,zd) in meters per second

	//! \brief  Default contructor.  All data members set to 0 or identity.
	AtlasPositionData() :
		position(0.0f),
		velocity(0.0f)
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
///namespace Atlas {

enum
{
	FS_LEFT  = 0,
	FS_RIGHT,
	NUM_FOOT_SENSORS
};

///}

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
///namespace Atlas {

enum
{
	WS_LEFT = 0,
	WS_RIGHT,
	NUM_WRIST_SENSORS
};

///}


///@}

#ifndef NOT_IN_DOXYGEN
} // end namespace Atlas
#endif


#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasCommonTypes_H

