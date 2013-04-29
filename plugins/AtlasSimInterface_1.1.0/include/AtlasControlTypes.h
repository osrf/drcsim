
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


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasJointId
//!
//!  \brief   Enumerations identifying robot joints.
//!
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


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasFootId
//!
//!  \brief   Enumerations identifying robot feet.
//!
typedef enum
{
	FOOT_LEFT         = 0,
	FOOT_RIGHT        = 1,
	NUM_FEET

} AtlasFootId;


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Behavior control types.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepData
//!
//!  \brief   Structure for desired foot step data.
//!
struct AtlasBehaviorStepData
{
	//!
	//!  \brief  Step index.
	//!
	//!    Start this at 1 at the beginning of Walk and Step behaviors, and
	//!    monotonically increase it for each step during walking.  Reset to 1
	//!    if the robot re-enters Walk or Step.
	//!
	//!    To specify "not a step", i.e., that an instance doesn't contain
	//!    data that should be considered a step, set the step_index to -1.
	//!
	int32_t step_index;

	//!
	//!  \brief  Foot index (Left/Right); see AtlasFootId enum.
	//!
	//!    For the Walk behavior, the foot index needs to alternate with each
	//!    step; consecutive steps with the same foot are not allowed.
	//!
	int32_t foot_index;

	//!
	//!  \brief  Step duration.
	//!
	float duration;

	//!
	//!  \brief  Foot position, in Atlas world frame.
	//!
	AtlasVec3f position;

	//!
	//!  \brief  Foot orientation yaw component, in Atlas world frame.
	//!
	float yaw;
	

	///////////////////////////////////////////////////////////
	//! ** Currently Unused Parameters -- Subject to change **
	///////////////////////////////////////////////////////////

	// Foot ground normal.
	AtlasVec3f normal;      //!<  (currently unused)

	// Step apex swing height as measured from the midpoint between the feet.
	float swing_height;     //!<  (currently unused)

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorStepData() :
		step_index(-1),
		foot_index(0),
		duration(0.7f),
		position(0.0f, 0.0f, 0.0f),
		yaw(0.0f),
		normal(0.0f, 0.0f, 1.0f),
		swing_height(0.0f)
	{}

	//!
	//!  \brief  Alternate constructor, when all parameters are known.
	//!
	AtlasBehaviorStepData(int32_t _step_index,
		int32_t    _foot_index,
		float      _duration,
		AtlasVec3f _position,
		float      _yaw,
		AtlasVec3f _normal,
		float      _swing_height)
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
//!  \struct    AtlasBehaviorPelvisServoParams
//!
//!  \brief   Structure for parameters controlling pelvis servo.
//!
struct AtlasBehaviorPelvisServoParams
{
	//!
	//!  \brief  User desired height of the pelvis.
	//!
	float pelvis_height;

	//!
	//!  \brief  User desired yaw angle of the pelvis.
	//!
	//!    Centered about the average yaw of the feet.
	//!
	float pelvis_yaw;

	//!
	//!  \brief  User desired horizontal offset of the COM.
	//!
	//!    In the direction of the vector between the two feet, centered about
	//!    the average position of the two feet.
	//!
	float pelvis_lat;

	AtlasBehaviorPelvisServoParams() :
		pelvis_height(0.8f),
		pelvis_yaw(0.0f),
		pelvis_lat(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStandParams
//!
//!  \brief   Structure for parameters for the Stand behavior.
//!
struct AtlasBehaviorStandParams
{
	int placeholder;

	AtlasBehaviorStandParams() :
		placeholder(0)
		{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStandFeedback
//!
//!  \brief   Structure for feedback data specific for Stand
//!
struct AtlasBehaviorStandFeedback
{
	//!
	//!  \brief  Bit-wise array of AtlasBehaviorStandFlags values
	//!          indicating Stand behavior status.
	//!
	//!    See documentation for AtlasBehaviorStandFeedback::status_flags
	//!    for information on how the flags work and example code of how to
	//!    use them.
	//!
	uint32_t status_flags;

	AtlasBehaviorStandFeedback() :
		status_flags(0)
	{}
};

//!
//!  \enum  AtlasBehaviorStandFlags
//!
typedef enum
{
	STAND_OKAY                  = 0,
	STAND_FLAG_PLACEHOLDER      = 1 << 1

} AtlasBehaviorStandFlags;


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepParams
//!
//!  \brief   Structure for parameters for the Step behavior.
//!
struct AtlasBehaviorStepParams
{
	//!
	//!  \brief  Desired position of next step.
	//!
	//!    The first desired step location should be set before entering
	//!    the Step behavior.
	//!
	//!    Refer to the AtlasSimInterface API Guide for details on when and
	//!    how subsequent steps can be queued.
	//!
	AtlasBehaviorStepData desired_step;

	//!
	//!  \brief  Whether to use demo walk for step data.
	//!
	bool use_demo_walk;

	AtlasBehaviorStepParams() :
		use_demo_walk(true)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepFeedback
//!
//!  \brief   Structure for feedback data specific to the Step behavior.
//!
struct AtlasBehaviorStepFeedback
{

	//!
	//!  \brief  Estimated step time remaining before new step data is needed.
	//!
	float t_step_rem;

	//!
	//!  \brief  Current step index.
	//!
	//!    Step index 0 is the position of the foot at the beginning of Step
	//!    behavior.  Step index 1 is the first user-specified foot position,
	//!    etc.
	//!
	//!    The step index will increase as Step progresses.  It will reset to
	//!    0 on behavior changes.
	//!
	int32_t current_step_index;

	//!
	//!  \brief  Next step index needed.
	//!
	//!    At the next touchdown, Walk will be looking for step data with
	//!    indices starting with next_step_index_needed.  In general this will
	//!    be current_step_index + 1.
	//!
	//!    This variable should be watched to know when the next step should
	//!    be queued.
	//!
	int32_t next_step_index_needed;

	//!
	//!  \brief  Bit-wise array of AtlasBehaviorStepFlags values indicating
	//!          Walk behavior status.
	//!
	//!    All flags are 0 when the behavior begins, and will be set to 1
	//!    at appropriate times.
	//!
	//!    All flags will latch unless otherwise noted.  "Latch" means that
	//!    once the flag is set to 1, it won't be re-set to 0 until the
	//!    behavior is left and re-entered.
	//!
	//!    \e Flags:
	//!
	//!    - STEP_SUBSTATE_SWAYING
	//!
	//!        Feet are in double support.  This flag does not latch.  Only
	//!        one of STEP_SUBSTATE_SWAYING or STEP_SUBSTATE_STEPPING will be
	//!        set at any given time.
	//!
	//!    - STEP_SUBSTATE_STEPPING
	//!
	//!        Actively stepping; one foot is in the air.  This flag does not
	//!        latch.
	//!
	uint32_t status_flags;

	//!
	//!  \brief  Saturated copy of desired_step from user.
	//!
	//!    Step will do its best to reach the desired step data, but may have
	//!    to modify data to fit within constraints.  The modified data is
	//!    contained in this variable.
	//!
	AtlasBehaviorStepData desired_step_saturated;

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorStepFeedback() :
	t_step_rem(0.0f),
		current_step_index(0),
		next_step_index_needed(1),
		status_flags(0)
	{}
};

//!
//!  \enum  AtlasBehaviorStepFlags
//!
//!    Descriptions of the flags are in the AtlasBehaviorStepFeedback
//!    status_flags variable.
//!
typedef enum
{
	STEP_OKAY                  = 0,
	STEP_SUBSTATE_SWAYING      = 1 << 0,
	STEP_SUBSTATE_STEPPING     = 1 << 1

} AtlasBehaviorStepFlags;


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorWalkParams
//!
//!  \brief   Structure for parameters for the Walk behavior.
//!
#define NUM_REQUIRED_WALK_STEPS 4

struct AtlasBehaviorWalkParams
{
	//!
	//!  \brief  Positions of upcoming steps.
	//!
	//!    Steps in the array whose step_index variables are set to -1 are
	//!    considered to not contain step data.
	//!
	//!    Refer to the AtlasSimInterface API Guide for details on when and
	//!    how steps can be queued.
	//!
	AtlasBehaviorStepData step_queue[NUM_REQUIRED_WALK_STEPS];

	//!
	//!  \brief  Whether to use demo walk for step data.
	//!
	bool use_demo_walk;

	AtlasBehaviorWalkParams() :
		use_demo_walk(true)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorWalkFeedback
//!
//!  \brief   Structure for feedback data specific to the Walk behavior.
//!
struct AtlasBehaviorWalkFeedback
{
	//!
	//!  \brief  Estimated step time remaining.
	//!
	float t_step_rem;

	//!
	//!  \brief  Current step index.
	//!
	//!    Step 0 is the position of the foot at the beginning of Walk.  Step
	//!    1 is the first user-specified foot position, etc.
	//!
	//!    The step index will increase as Walk progresses.  It will reset to
	//!    0 on behavior changes.
	//!
	int32_t current_step_index;

	//!
	//!  \brief  Next step index needed.
	//!
	//!    At the next touchdown, Walk will be looking for step data with
	//!    indices starting with next_step_index_needed.  This will usually
	//!    be roughly current_step_index plus four.
	//!
	//!    This variable should be watched to know when the next step should
	//!    be queued.
	//!
	int32_t next_step_index_needed;

	//!
	//!  \brief  Bit-wise array of AtlasBehaviorWalkFlags values indicating
	//!          Walk behavior status.
	//!
	//!    All flags are 0 when the behavior begins, and will be set to 1
	//!    at appropriate times.
	//!
	//!    All flags will latch unless otherwise noted.  "Latch" means that
	//!    once the flag is set to 1, it won't be re-set to 0 until the
	//!    behavior is left and re-entered.
	//!
	//!    To check a flag, use something like:
	//!
	//!    <pre>
	//!       if (walk_feedback.status_flags & WALK_ERROR_INCONSISTENT_STEPS)
	//!           do_something_about_the_error();
	//!    </pre>
	//!
	//!    \e Flags:
	//!
	//!    - WALK_SUBSTATE_SWAYING
	//!
	//!        Walk is preparing to take a its first step by swaying the body.
	//!        This flag does not latch.  Only one of WALK_SUBSTATE_SWAYING,
	//!        WALK_SUBSTATE_STEPPING, or WALK_SUBSTATE_CATCHING will be set
	//!        at any given time.
	//!
	//!    - WALK_SUBSTATE_STEPPING
	//!
	//!        Walk is actively stepping.  This flag does not latch.
	//!
	//!    - WALK_SUBSTATE_CATCHING
	//!
	//!        Walk has stopped stepping and is preparing to go back to Stand.
	//!        This flag does not latch.
	//!
	//!    - WALK_WARNING_INSUFFICIENT_STEP_DATA
	//!
	//!        Walk was unable to find the number of steps it was looking for
	//!        and has triggered an attempt to go back to Stand.  (Does not
	//!        latch.)
	//!
	//!    - WALK_ERROR_INCONSISTENT_STEPS
	//!
	//!        Walk detected the user tried to take 2 steps with the same
	//!        foot, and attempt to go back into Stand.
	//!
	uint32_t status_flags;

	//!
	//!  \brief  Saturated copy of step_queue from user.
	//!
	//!    Walk will do its best to reach the desired step data, but may have
	//!    to modify data to fit within constraints.  The modified data is
	//!    contained in this variable.
	//!
	AtlasBehaviorStepData step_queue_saturated[NUM_REQUIRED_WALK_STEPS];

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorWalkFeedback() :
		t_step_rem(0.0f),
		current_step_index(0),
		next_step_index_needed(1),
		status_flags(0)
	{}
};

//!
//!  \enum  AtlasBehaviorWalkFlags
//!
//!    Descriptions of the flags are in the AtlasBehaviorWalkFeedback
//!    status_flags variable.
//!
typedef enum
{
	WALK_OKAY                            = 0,
	WALK_SUBSTATE_SWAYING                = 1 << 0,
	WALK_SUBSTATE_STEPPING               = 1 << 1,
	WALK_SUBSTATE_CATCHING               = 1 << 2,
	WALK_WARNING_INSUFFICIENT_STEP_DATA  = 1 << 3,
	WALK_ERROR_INCONSISTENT_STEPS        = 1 << 4

} AtlasBehaviorWalkFlags;


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorManipulateParams
//!
//!  \brief   Structure for parameters for the Manipulate behavior.
//!
struct AtlasBehaviorManipulateParams
{
	//!
	//!  \brief  Whether to use user provided desired pelvis servo params.
	//!
	bool use_desired;

	//!
	//!  \brief  User desired pelvis servo params.
	//!
	AtlasBehaviorPelvisServoParams desired;

	//!
	//!  \brief  Whether to drive manipulate params with demo data.
	//!
	bool use_demo_mode;

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorManipulateParams() :
		use_desired(true)
		{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorManipulateFeedback
//!
//!  \brief   Structure for feedback data specific for Manipulate.
//!
struct AtlasBehaviorManipulateFeedback
{
	//!
	//!  \brief  Bit-wise array of AtlasBehaviorManipulateFlags values
	//!          indicating Manipulate behavior status.
	//!
	//!    See documentation for AtlasBehaviorWalkFeedback::status_flags
	//!    for information on how the flags work and example code of how to
	//!    use them.
	//!
	uint32_t status_flags;

	//!
	//!  \brief  ...
	//!
	AtlasBehaviorPelvisServoParams clamped;

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorManipulateFeedback() :
		status_flags(0)
	{}
};

//!
//!  \enum  AtlasBehaviorManipulateFlags
//!
typedef enum
{
	MANIPULATE_OKAY                  = 0,
	MANIPULATE_FLAG_PLACEHOLDER      = 1 << 1

} AtlasBehaviorManipulateFlags;


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorFeedback
//!
//!  \brief   Structure for returning errors and other information related to behaviors.
//!
struct AtlasBehaviorFeedback
{
	//!
	//!  \brief  Bit-wise array of AtlasBehaviorFlags values indicating
	//!          non-specific-behavior status.
	//!
	//!    See documentation for AtlasBehaviorWalkFeedback::status_flags
	//!    for information on how the flags work and example code of how to
	//!    use them.
	//!
	//!  \em Transition flags:
	//!
	//!    - STATUS_TRANSITION_IN_PROGRESS
	//!
	//!        A transition is in progress.
	//!
	//!    - STATUS_TRANSITION_SUCCESS
	//!
	//!        Successful transition.
	//!
	//!    - STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR
	//!
	//!        Failed to transition; unknown behavior.
	//!
	//!    - STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR
	//!
	//!        Denied request for an illegal behavior transition.  This may
	//!        happen if a transition to a new behavior is requested without
	//!        going through a required intermediate behavior. (e.g., can't
	//!        go from Walk straight to Manipulate.)
	//!
	//!    - STATUS_FAILED_TRANS_COM_POS
	//!
	//!        Failed to transition; the position of the COM is too far from
	//!        the center of support.
	//!
	//!    - STATUS_FAILED_TRANS_COM_VEL
	//!
	//!        Failed to transition; the COM velocity too high.
	//!
	//!    - STATUS_FAILED_TRANS_VEL
	//!
	//!        Failed to transition; some joint velocities too high.
	//!
	//!  \em Warnings:
	//!
	//!    - STATUS_WARNING_AUTO_TRANS
	//!
	//!        An automatic transition occurred; see behavior specific
	//!        feedback for reason.
	//!
	//!  \em Errors:
	//!
	//!    - STATUS_ERROR_FALLING
	//!
	//!        COM below acceptable threshold, cannot recover.
	//!
	uint32_t status_flags;

	//!
	//!  \brief  Index of the behavior from which a transition is being
	//!          attempted.
	//!
	//!    The string name of the behavior can be looked up by calling
	//!    AtlasSimInterface::get_behavior_at_index().
	//!
	int trans_from_behavior_index;

	//!
	//!  \brief  Index of the behavior to which a transition is being
	//!          attempted.
	//!
	//!    The string name of the behavior can be looked up by calling
	//!    AtlasSimInterface::get_behavior_at_index().
	//!
	int trans_to_behavior_index;

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorFeedback() :
		status_flags(0l),
		trans_from_behavior_index(-1),
		trans_to_behavior_index(-1)
	{}
};

//!
//!  \enum  AtlasBehaviorFlags
//!
//!    Descriptions of the flags are with the AtlasBehaviorFeedback
//!    status_flags variable.
//!
typedef enum
{
	STATUS_OK = 0,       //!<  Normal operation, nothing to report.

	STATUS_TRANSITION_IN_PROGRESS        = 1 << 0,
	STATUS_TRANSITION_SUCCESS            = 1 << 1,
	STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR = 1 << 2,
	STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR = 1 << 3,
	STATUS_FAILED_TRANS_COM_POS          = 1 << 4,
	STATUS_FAILED_TRANS_COM_VEL          = 1 << 5,
	STATUS_FAILED_TRANS_VEL              = 1 << 6,

	STATUS_WARNING_AUTO_TRANS            = 1 << 7,
	STATUS_ERROR_FALLING                 = 1 << 8

} AtlasBehaviorFlags;


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name From-robot types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasIMUData
//!
//!  \brief   Structure for returned IMU readings.
//!
//!    All angular velocities and linear accelerations are expressed relative
//!    to an intertial frame, with respect to the IMU frame.
//!
//!    The orientation estimate is expressed relative to an intertial frame,
//!    with respect to the IMU frame.  Note: the z axis of the inertial frame
//!    is aligned with the gravity vector but is opposite in sign.
//!
struct AtlasIMUData
{
	//!
	//!  \brief  Timestamp for IMU data [s*(10^-6)]
	//!
	uint64_t imu_timestamp;

	//!
	//!  \brief  Approximate orientation, as quaternion.
	//!
	AtlasQuaternion orientation_estimate;

	//!
	//!  \brief  Angular velocity about x, y, z axes, in rad/s.
	//!
	AtlasVec3f angular_velocity;

	//!
	//!  \brief  Linear acceleration along x, y, z axes, in m/s^2.
	//!
	AtlasVec3f linear_acceleration;

	//!
	//!  \brief  Default contructor.  All data members set to 0 or identity.
	//!
	AtlasIMUData() :
		imu_timestamp(0l),
		angular_velocity(0.0f),
		linear_acceleration(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasPositionData
//!
//!  \brief   Structure for returned Atlas robot state estimates
//!
//!    All positions and velocities expressed in the world frame as sensed by
//!    the IMU.
//!
struct AtlasPositionData
{
	//!
	//!  \brief  Position estimate of robot pelvis (x,y,z) in meters.
	//!
	AtlasVec3f      position;

	//!
	//!  \brief  Velocity estimate of robot pelvis (xd,yd,zd) in meters per second.
	//!
	AtlasVec3f      velocity;

	//!
	//!  \brief  Default contructor.  All data members set to 0 or identity.
	//!
	AtlasPositionData() :
		position(0.0f),
		velocity(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasFootSensor
//!
//!  \brief   Structure for returned foot sensors.
//!
//!    This structure holds the force and moments exterted on a foot by the
//!    environment, in the foot's coordinate frame.
//!
struct AtlasFootSensor
{
	//!
	//!  \brief  Force exerted on foot along z axis, in N.
	//!
	float fz;

	//!
	//!  \brief  Moment exerted on foot about x axis, in N*m.
	//!
	float mx;

	//!
	//!  \brief  Moment exerted on foot about y axis, in N*m.
	//!
	float my;

	//!
	//!  \brief  Default contructor.  All data members set to 0.
	//!
	AtlasFootSensor() {fz = mx = my = 0.0f;}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasFootSensorId
//!
//!  \brief   Enumerations identifying robot foot sensors.
//!
enum
{
	FS_LEFT  = 0,
	FS_RIGHT,
	NUM_FOOT_SENSORS
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasWristSensor
//!
//!  \brief   Structure for returned wrist sensors.
//!
//!    This structure holds the forces and moments exterted on a hand by
//!    the environment, in the hand's coordinate frame.
//!
struct AtlasWristSensor
{
	//!
	//!  \brief  Forces exerted on hand along x, y, z axes, in N.
	//!
	AtlasVec3f f;

	//!
	//!  \brief  Moments exerted on hand about x, y, z axes, in N*m.
	//!
	AtlasVec3f m;

	//!
	//!  \brief  Default contructor.  All data members set to 0.
	//!
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
enum
{
	WS_LEFT = 0,
	WS_RIGHT,
	NUM_WRIST_SENSORS
};


///@}

#ifndef NOT_IN_DOXYGEN
} // end namespace Atlas
#endif


#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasCommonTypes_H

