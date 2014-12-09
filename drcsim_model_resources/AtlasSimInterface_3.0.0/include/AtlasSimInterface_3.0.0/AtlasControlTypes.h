
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

#ifdef __GNUC__
#pragma pack(push,1)
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
	JOINT_BACK_BKZ    = 0,
	JOINT_BACK_BKY    = 1,
	JOINT_BACK_BKX    = 2,
	JOINT_NECK_AY     = 3,
	JOINT_L_LEG_HPZ   = 4,
	JOINT_L_LEG_HPX   = 5,
	JOINT_L_LEG_HPY   = 6,
	JOINT_L_LEG_KNY   = 7,
	JOINT_L_LEG_AKY   = 8,
	JOINT_L_LEG_AKX   = 9,
	JOINT_R_LEG_HPZ   = 10,
	JOINT_R_LEG_HPX   = 11,
	JOINT_R_LEG_HPY   = 12,
	JOINT_R_LEG_KNY   = 13,
	JOINT_R_LEG_AKY   = 14,
	JOINT_R_LEG_AKX   = 15,
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
//!  \enum    AtlasActuatorId
//!
//!  \brief   Enumerations identifying robot actuators.
//!
typedef enum
{
	ACTUATOR_UNKNOWN     = -1,
	ACTUATOR_BACK_BKZ    = 0,
	ACTUATOR_BACK_BKY    = 1,
	ACTUATOR_BACK_BKX    = 2,
	ACTUATOR_NECK_AY     = 3,
	ACTUATOR_L_LEG_HPZ   = 4,
	ACTUATOR_L_LEG_HPX   = 5,
	ACTUATOR_L_LEG_HPY   = 6,
	ACTUATOR_L_LEG_KNY   = 7,
	ACTUATOR_L_LEG_LAK   = 8,
	ACTUATOR_L_LEG_RAK   = 9,
	ACTUATOR_R_LEG_HPZ   = 10,
	ACTUATOR_R_LEG_HPX   = 11,
	ACTUATOR_R_LEG_HPY   = 12,
	ACTUATOR_R_LEG_KNY   = 13,
	ACTUATOR_R_LEG_LAK   = 14,
	ACTUATOR_R_LEG_RAK   = 15,
	ACTUATOR_L_ARM_USY   = 16,
	ACTUATOR_L_ARM_SHX   = 17,
	ACTUATOR_L_ARM_ELY   = 18,
	ACTUATOR_L_ARM_ELX   = 19,
	ACTUATOR_L_ARM_UWY   = 20,
	ACTUATOR_L_ARM_MWX   = 21,
	ACTUATOR_R_ARM_USY   = 22,
	ACTUATOR_R_ARM_SHX   = 23,
	ACTUATOR_R_ARM_ELY   = 24,
	ACTUATOR_R_ARM_ELX   = 25,
	ACTUATOR_R_ARM_UWY   = 26,
	ACTUATOR_R_ARM_MWX   = 27,
	NUM_ACTUATORS

} AtlasActuatorId;

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


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasHandId
//!
//!  \brief   Enumerations identifying robot hands.
//!
typedef enum
{
	HAND_LEFT         = 0,
	HAND_RIGHT        = 1,
	NUM_HANDS

} AtlasHandId;


///////////////////////////////////////////////////////////
//!
//!  \enum    ToeOffSetting
//!
//!  \brief   Enumerations specifying toe-off behavior
//!
typedef enum {
	//! \brief disable toe-off
	TOE_OFF_DISABLE,
	//! \brief enable toe-off as needed
	TOE_OFF_ENABLE,
	//! \brief force a toe-off
	TOE_OFF_FORCE_ENABLE
} ToeOffSetting;

///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Behavior control types.
//!
///@{

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorFootData
//!
//!  \brief   Structure for foot hold description.
//!
struct AtlasBehaviorFootData
{
	//!
	//!  \brief  Foot position, in Atlas world frame.
	//!
	AtlasVec3f position;

	//!
	//!  \brief  Foot orientation yaw component, in Atlas world frame.
	//!
	float yaw;

	//!
	//!  \brief  Foot ground normal, in Atlas world frame.
	//!
	AtlasVec3f normal;

	AtlasBehaviorFootData():
		position(0.0f, 0.0f, 0.0f),
		yaw(0.0f),
		normal(0.0f, 0.0f, 1.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepAction
//!
//!  \brief   parameters for a single step in the Step behavior
//!
struct AtlasBehaviorStepAction
{
	//!
	//! \brief duration of the single support phase (STEP_SUBSTATE_STEPPING)
	//! 
	float step_duration;

	//!
	//! \brief duration of the double support phase (STEP_SUBSTATE_SWAYING)
	//! 
	//! Ignored if it is less than 0.2
	//!
	float sway_duration;

	//!
	//! \brief step apex above the lift height while swinging the leg
	//! 
	//! This is the additional height above the lift height that the foot 
	//! will reach while swinging to its final x,y position
	//! 
	//! Ignored if it is less than 0.1
	//!
	float swing_height;

	//!
	//! \brief height to lift vertically before moving the foot in x,y
	//! 
	//! This is computed to be above the maximum of the start 
	//! and end locations of the foot.
	//! 
	//! Ignored if it is less than 0.1
	//!
	float lift_height;

	//!
	//! \brief specify whether toe-off is allowed or not during swaying
	//! 
	//! Should be set to TOE_OFF_DISABLE, TOE_OFF_ENABLE, or TOE_OFF_FORCE_ENABLE
	//! For feedback, this will be set to TOE_OFF_DISABLE or TOE_OFF_ENABLE, based
	//! on whether the step is using a toe-off or not.
	//!
	int32_t toe_off;

	//!
	//! \brief Nominal knee angle during the step.
	//! 
	//! Used to change the desired amount of knee flex while walking.
	//! Useful to tweak when running into ankle limits on sloped terrain.
	//! 
	//! Ignored if less than 0.3
	//! 
	float knee_nominal;

	//!
	//! \brief Maximum body acceleration to determine minimum sway duration
	//! 
	//! The minimum sway duration will be determined by this maximum allowable
	//! body acceleration.  Increase this value to allow the robot to move
	//! faster, but doing so may cause instability.
	//! 
	//! Ignored if zero
	//! 
	float max_body_accel;

	//!
	//! \brief Maximum foot velocity to determine minimum step duration
	//! 
	//! The minimum step duration will be determined by this maximum
	//! foot velocity.  Increase this value to allow the robot to move faster,
	//! but doing so may destabilize the robot.
	//! 
	//! Ignored if zero
	//! 
	float max_foot_vel;

	//!
	//! \brief Distance short of the foot to aim for at the end of sway (in meters)
	//! 
	//! Tuning parameter for swaying.  Increase this value if the robot 
	//! falls to the outside at the end of a sway. 
	//! 
	//! Ignored if not less than +- 5cm
	//! 
	float sway_end_dist;

	//!
	//! \brief Distance to lean into the step before the foot comes down (in meters)
	//! 
	//! Tuning parameter for stepping.  Increase this value if the foot 
	//! touchdown tends push the robot over unstably.
	//! 
	//! Ignored if not less than +- 5cm
	//! 
	float step_end_dist;

	AtlasBehaviorStepAction():
		step_duration(0.7f),sway_duration(0.1f),
		swing_height(0.0f), lift_height(0.0f), toe_off(TOE_OFF_ENABLE),
		knee_nominal(0.0f), max_body_accel(0.0f),
		max_foot_vel(0.0f), sway_end_dist(0.1f), step_end_dist(0.1f)
	{}
};

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorWalkAction
//!
//!  \brief   parameters for a single step in the Walk behavior
//!
struct AtlasBehaviorWalkAction
{
	//!
	//! \brief Step duration
	//! 
	float step_duration;

	//!
	//!  \brief  Step apex swing height as measured from the midpoint between the feet.
	//!
	float swing_height;

	AtlasBehaviorWalkAction():
		step_duration(0.7f), swing_height(0.0f)
	{}
};

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepSpec
//!
//!  \brief   Structure for stepping data for Step.
//!
struct AtlasBehaviorStepSpec
{
	//!
	//!  \brief  Step index.
	//!
	//!    Start this at 1 at the beginning of the Step behavior, and
	//!    monotonically increase it for each step during walking.  Reset to 1
	//!    if the robot re-enters Step.
	//!
	//!    To specify "not a step", i.e., that an instance doesn't contain
	//!    data that should be considered a step, set the step_index to -1.
	//!
	int32_t step_index;

	//!
	//!  \brief  Foot index (Left/Right); see AtlasFootId enum.
	//!
	int32_t foot_index;

	//!
	//!  \brief  Foothold to step to
	//!
	AtlasBehaviorFootData foot;

	//!
	//!  \brief  Parameters of the stepping action
	//!
	AtlasBehaviorStepAction action;
};

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorWalkSpec
//!
//!  \brief   Structure for stepping data for Walk.
//!
struct AtlasBehaviorWalkSpec
{
	//!
	//!  \brief  Step index.
	//!
	//!    Start this at 1 at the beginning of the Walk behaviors, and
	//!    monotonically increase it for each step during walking.  Reset to 1
	//!    if the robot re-enters Walk.
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
	//!  \brief  Foothold to step to
	//!
	AtlasBehaviorFootData foot;

	//!
	//!  \brief  Parameters of the stepping action
	//!
	AtlasBehaviorWalkAction action;
};

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasBehaviorStepData
//!
//!  \brief   Deprecated structure for desired foot step data.  Please use
//!  AtlasBehaviorStepSpec or AtlasBehaviorWalkSpec instead  
//!  
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
	
	//!
	//!  \brief  Foot ground normal, in Atlas world frame.
	//!
	AtlasVec3f normal;

	//!
	//!  \brief  Step apex swing height as measured from the midpoint between the feet.
	//!
	float swing_height;

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
	//!  \brief  User desired pitch angle of the pelvis.
	//!
	float pelvis_pitch;

	//!
	//!  \brief  User desired roll angle of the pelvis.
	//!
	float pelvis_roll;

	//!
	//!  \brief  User desired horizontal offset of the COM.
	//!
	//!    In the direction of the vector between the two feet, centered about
	//!    the average position of the two feet.
	//!
	float com_v0;

	//!
	//!  \brief  User desired horizontal offset of the COM.
	//!
	//!    In the direction perpendicular to the vector between the two feet,
	//!    centered about the average position of the two feet.
	//!
	float com_v1;

	AtlasBehaviorPelvisServoParams() :
		pelvis_height(0.8f),
		pelvis_yaw(0.0f),
		pelvis_pitch(0.0f),
		pelvis_roll(0.0f),
		com_v0(0.0f),
		com_v1(0.0f)
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
	int32_t placeholder;

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
//!    This structure contains parameters needed for controlling the Step
//!    behavior.  Refer to the Atlas Software and Control Manual for details
//!    on when and how parameters should be set.
//!
struct AtlasBehaviorStepParams
{
	//!
	//!  \brief  Desired position of next step. desired_step_spec is preferred.
	//!
	//!    The first desired step location should be set before entering
	//!    the Step behavior.
	//!
	AtlasBehaviorStepData desired_step;

	//!
	//!  \brief  Desired position of next step.  Preferred method.
	//!
	//!    The first desired step location should be set before entering
	//!    the Step behavior.
	//!
	AtlasBehaviorStepSpec desired_step_spec;

	//!
	//!  \brief  use the extended step specification
	//!
	//!    Use the desired_step_spec data, instead of desired_step
	//!
	int32_t use_spec;

	//!
	//!  \brief  Whether to use relative step height.
	//!
	//!    If use_relative_step_height is true, step to step height variations
	//!    will be relative to the current stance foot height.  Otherwise the
	//!    foot will try to step relative to the BD world position estimate
	//!    which can be subject to drift.
	//!
	int32_t use_relative_step_height;

	//!
	//!  \brief  Whether to use demo walk for step data.
	//!
	int32_t use_demo_walk;

	//!
	//! \brief offset orientation for the pelvis
	//! 
	//! Sets a target orientation offset for the pelvis during the
	//! stepping.  Setting the offset too large may interfere with the 
	//! step motion due to joint range of motion or strength limits.
	//! This value is read at the beginning of each stepping phase.
	//! 
	AtlasQuaternion pelvis_orientation_offset;

	AtlasBehaviorStepParams() :
		use_spec(false),
		use_relative_step_height(false),
		use_demo_walk(false)
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
	//!  \brief  Saturated copy of desired_step_spec from user.
	//!
	//!    Step will do its best to reach the desired step data, but may have
	//!    to modify data to fit within constraints.  The modified data is
	//!    contained in this variable.
	//!
	AtlasBehaviorStepSpec desired_step_spec_saturated;

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
	//!  \brief  Positions of upcoming steps. walk_spec_queue is preferred
	//!
	//!    Steps in the array whose step_index variables are set to -1 are
	//!    considered to not contain step data.
	//!
	//!    Refer to the AtlasRobotInterface or AtlasSimInterface API Guide for
	//!    details on when and how steps can be queued.
	//!
	AtlasBehaviorStepData step_queue[NUM_REQUIRED_WALK_STEPS];
	//!
	//!  \brief  Positions of upcoming steps.  Preferred method.
	//!
	//!    Steps in the array whose step_index variables are set to -1 are
	//!    considered to not contain step data.
	//!
	//!    Refer to the AtlasRobotInterface or AtlasSimInterface API Guide for
	//!    details on when and how steps can be queued.
	//!
	AtlasBehaviorWalkSpec walk_spec_queue[NUM_REQUIRED_WALK_STEPS];

	//!
	//!  \brief  use the extended step specification
	//!
	//!    Use the walk_spec_queue data, instead of step_queue
	//!
	int32_t use_spec;

	//!
	//!  \brief  Whether to use relative step height.
	//!
	//!    If use_relative_step_height is true, step to step height variations
	//!    will be relative to the current stance foot height.  Otherwise the
	//!    foot will try to step relative to the BD world position estimate
	//!    which can be subject to drift.
	//!
	int32_t use_relative_step_height;

	//!
	//!  \brief  Whether to use demo walk for step data.
	//!
	int32_t use_demo_walk;

	AtlasBehaviorWalkParams() :
		use_spec(false),
		use_relative_step_height(false),
		use_demo_walk(false)
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
	//!  \brief  Saturated copy of walk_spec_queue from user.
	//!
	//!    Walk will do its best to reach the desired step data, but may have
	//!    to modify data to fit within constraints.  The modified data is
	//!    contained in this variable.
	//!
	AtlasBehaviorWalkSpec walk_spec_queue_saturated[NUM_REQUIRED_WALK_STEPS];

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
	int32_t use_desired;

	//!
	//!  \brief  User desired pelvis servo params.
	//!
	AtlasBehaviorPelvisServoParams desired;

	//!
	//!  \brief  Whether to drive manipulate params with demo data.
	//!
	int32_t use_demo_mode;

	//!
	//!  \brief  Default constructor.
	//!
	AtlasBehaviorManipulateParams() :
		use_desired(true),
		use_demo_mode(0)
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
	//!  \brief  Clamped version of the user desired set-point
	//!
	AtlasBehaviorPelvisServoParams clamped;

	//!
	//!  \brief  Current internal desired values after processing
	//!
	AtlasBehaviorPelvisServoParams internal_desired;

	//!
	//!  \brief  Current internal sensed values
	//!
	AtlasBehaviorPelvisServoParams internal_sensed;

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
	//!    If using the AtlasRobotInterface, this index corresponds
	//!    numerically with a AtlasRobotBehavior enumeration, and can be cast
	//!    as such.
	//!
	//!    The using the AtlasSimInterface, the string name of the behavior
	//!    can be looked up by calling
	//!    AtlasSimInterface::get_behavior_at_index().
	//!
	int trans_from_behavior_index;

	//!
	//!  \brief  Index of the behavior to which a transition is being
	//!          attempted.
	//!
	//!    See trans_from_behavior_index for what the index means.
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
	STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR = 1 << 2,
	STATUS_FAILED_TRANS_COM_POS          = 1 << 3,
	STATUS_FAILED_TRANS_COM_VEL          = 1 << 4,
	STATUS_FAILED_TRANS_VEL              = 1 << 5,
	STATUS_WARNING_AUTO_TRANS            = 1 << 6,
	STATUS_ERROR_FALLING                 = 1 << 7 

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
//!    to an inertial frame, with respect to the IMU frame.
//!
//!    The IMU is centered a (X: -90.5mm, Y: -0.004mm, Z: -12.5mm) with
//!    respect to the robot origin and rotated by 45 degrees along the z-axis
//!    (vertical).  The z-axis of the IMU coordinate frame points in the
//!    downward direction (opposite the robot convention).  Each linear
//!    accelerometer is offset by a different fixed translation in the IMU
//!    frame.
//!   
//!
struct AtlasIMUData
{
#ifndef ATLAS3_API_SIM_INTERFACE
	int64_t seq_id;
#endif

	//!
	//!  \brief  Timestamp for IMU data (microseconds)
	//!
	uint64_t imu_timestamp;

	//!
	//!  \brief  Approximate orientation (quaternion) of the pelvis frame w.r.t. the inertial frame.
	//!
	AtlasQuaternion orientation_estimate;

	//!
	//!  \brief  Angular velocity (rad/s) of the pelvis frame w.r.t. the inertial frame.
	//!
	AtlasVec3f angular_velocity;

	//!
	//!  \brief Linear acceleration (m/s^2) in the frame of the IMU. The location of the sensor differs in each direction. (See IMU details above.)  
	//!
	AtlasVec3f linear_acceleration;

	//!
	//!  \brief  Default constructor.  All data members set to 0 or identity.
	//!
	AtlasIMUData() :
		imu_timestamp(0l),
		angular_velocity(0.0f),
		linear_acceleration(0.0f)
	{
#ifndef ATLAS3_API_SIM_INTERFACE
		seq_id = 0;
#endif
	}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasRawIMUData
//!
//!  \brief   Structure for returned raw IMU readings.
//!
enum
{
	NUM_RAW_IMU_PACKETS = 15
};

struct AtlasRawIMUData
{

	//!
	//!  \brief  Timestamp for IMU data [s*(10^-6)].
	//!
	uint64_t imu_timestamp;

	//!
	//!  \brief  Packet number from IMU.
	//!
	uint64_t packet_count;

	//!
	//!  \brief  Delta angle (radians) in the frame of the IMU. (See IMU details above.) 
	//!
	double dax, day, daz;

	//!
	//!  \brief Linear acceleration (m/s^2) in the frame of the IMU. The location of the sensor differs in each direction. (See IMU details above.)  
	//!
	double ddx, ddy, ddz;

	//!
	//!  \brief  Default constructor.  All data members set to 0 or identity.
	//!
	AtlasRawIMUData() :
		imu_timestamp(0l),
		packet_count(0l),
		dax(0.0),
		day(0.0),
		daz(0.0),
		ddx(0.0),
		ddy(0.0),
		ddz(0.0)
	{
	}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasPositionData
//!
//!  \brief   Structure for returned Atlas robot state estimates
//!
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
	//!  \brief  Default constructor.  All data members set to 0 or identity.
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
//!   This structure holds the vertical force and horizontal moments exerted on the foot expressed w.r.t
//!   the foot coordinate frame at a point located 39mm below the ankle joint. 
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
	//!  \brief  Default constructor.  All data members set to 0.
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
//!  \enum    AtlasFootStrainGaugeId
//!
//!  \brief   Enumerations identifying robot strain gauges.
//!
enum
{
	SG_FOOT_G0_LEFT = 0,
	SG_FOOT_G1_LEFT,
	SG_FOOT_G2_LEFT,
	SG_FOOT_G3_LEFT,
	SG_FOOT_G0_RIGHT,
	SG_FOOT_G1_RIGHT,
	SG_FOOT_G2_RIGHT,
	SG_FOOT_G3_RIGHT,
	NUM_FOOT_STRAIN_GAUGES
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
	//!  \brief  Default constructor.  All data members set to 0.
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
//!  \brief   Enumerations identifying robot wrist sensors.
//!
enum
{
	WS_LEFT = 0,
	WS_RIGHT,
	NUM_WRIST_SENSORS
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasWristStrainGaugeId
//!
//!  \brief   Enumerations identifying robot strain gauges.
//!
enum
{
	SG_WRIST_G0_LEFT = 0,
	SG_WRIST_G1_LEFT,
	SG_WRIST_G2_LEFT,
	SG_WRIST_G3_LEFT,
	SG_WRIST_G4_LEFT,
	SG_WRIST_G5_LEFT,
	SG_WRIST_G0_RIGHT,
	SG_WRIST_G1_RIGHT,
	SG_WRIST_G2_RIGHT,
	SG_WRIST_G3_RIGHT,
	SG_WRIST_G4_RIGHT,
	SG_WRIST_G5_RIGHT,
	NUM_WRIST_STRAIN_GAUGES
};


///@}

#ifndef NOT_IN_DOXYGEN
} // end namespace Atlas
#endif

#ifdef __GNUC__
#pragma pack(pop)
#endif


#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif // __AtlasCommonTypes_H

