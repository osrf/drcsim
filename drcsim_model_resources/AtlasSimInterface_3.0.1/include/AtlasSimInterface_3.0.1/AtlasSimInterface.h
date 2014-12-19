
#ifndef __AtlasSimInterface_H
#define __AtlasSimInterface_H

#include <string>

#include "AtlasSimInterfaceTypes.h"

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

using namespace Atlas;
using namespace AtlasSim;


//////////////////////////////////////////////////////////////////////////////
//!
//! \mainpage  Overview
//!
//!    Click the 'Classes' or 'Files' buttons to dive into the documentation.
//!    A good place to start would be the AtlasSimInterface class.
//!
//!    Other things to look at:
//!
//!     \li <a href="Changes.html">API Changes/Release Notes</a>
//!
//! \image html Atlas_v4.jpg
//!

class AtlasSimInterface;

extern "C" {

//////////////////////////////////////////////////////////////////////////////
//!
//!  \brief  Create a simulation interface object.
//!
//!  \_Description
//!
//!    This function creates an AtlasSimInterface object.  It should not be
//!    called more than once.  Multiple calls will return the same object.
//!
//!    The object should not be destroyed using delete, but instead by calling
//!    destroy_atlas_sim_interface().
//!
AtlasSimInterface* create_atlas_sim_interface();


//////////////////////////////////////////////////////////////////////////////
//!
//!  \brief  Destroy the simulation interface object created by create_atlas_sim_interface().
//!
void destroy_atlas_sim_interface();

}  // end extern "C"


//////////////////////////////////////////////////////////////////////////////
//!
//!  \class AtlasSimInterface AtlasSimInterface.h
//!
//!  \brief The primary interface for the DRC version of the Atlas Robot.
//!
class AtlasSimInterface
{
public:

	int get_version_major();  //!< return major version number
	int get_version_minor();  //!< return minor version number
	int get_version_point();  //!< return point version number

	///////////////////////////////////////////////////////
	//!
	//! \brief  Given control inputs and current behavior, calculate control outputs.
	//!
	//! \_Description
	//!
	//!    This function will calculate control outputs that should be sent to
	//!    the Atlas Robot simulation based on values passed in in the
	//!    control_input object, and previously set behavior and related
	//!    settings.  It is up to the caller to have set values in the
	//!    control_input object based on values from a simulation.
	//!
	//!    The values in control_output can then be applied as desired to the
	//!    simulation.
	//!
	//!    *NOTE:* If an error is returned (e.g., the simulation is not in a
	//!    behavior), the control output will not be valid, and should not be
	//!    applied to the simulation.
	//!
	//!    *ALSO NOTE:*  Currently control outputs based on behaviors will
	//!    almost always be just the desired forces, with desired positions
	//!    and gains set to 0.
	//!
	//! \_Parameters
	//!
	//!    \_in   control_input   - control inputs
	//!    \_in   robot_state     - current state of simulated robot
	//!    \_out  control_output  - new desired state of sim
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_SUCH_BEHAVIOR
	//!    \li   ERROR_TIME_RAN_BACKWARD
	//!
	AtlasErrorCode process_control_input(const AtlasControlInput& control_input,
		const AtlasRobotState& robot_state,
		AtlasControlOutput& control_output);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Reset the behavior control.
	//!
	//! \_Description
	//!
	//!    Currently calling this function is the same as calling
	//!    <tt>set_desired_behavior("Freeze")</tt>.
	//!
	//!    At some point this function should reset the control system such
	//!    that a new simulation run can be started with completely reset
	//!    control parameters.
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!
	AtlasErrorCode reset_control();


	///////////////////////////////////////////////////////
	//!
	//! \brief  Set the desired behavior of the sim.
	//!
	//! \_Description
	//!
	//!    This function will set the *desired* behavior of the Atlas Robot
	//!    simulation.
	//!
	//!    There are a number of reasons the simulation may not be able to
	//!    follow the desired behavior.  For example, most desired behaviors
	//!    will not be achievable if the robot has fallen.  Also, it may take
	//!    a little while for the robot to switch from its current behavior
	//!    to the desired one.
	//!
	//!    Setting the desired behavior to "User" should immediately return
	//!    all joint control to the performer.
	//!
	//!    Current supported behaviors should include:
	//!
	//!      - "User"         - no control from behavior; all values set by performer
	//!      - "Stand"        - static stand behavior
	//!      - "Walk"         - continuous dynamic walk behavior
	//!      - "Step"         - single step, quasi-static behavior
	//!      - "Manipulate"   - stand, but with some user-controlled joints
	//!
	//!    All supported behaviors can be enumerated by calling
	//!    get_num_behaviors() and get_behavior_at_index().
	//!
	//! \_Parameters
	//!
	//!    \_in   behavior - name of desired behavior
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_SUCH_BEHAVIOR
	//!
	AtlasErrorCode set_desired_behavior(const std::string& behavior);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get the desired behavior of the sim.
	//!
	//! \_Description
	//!
	//!    This function returns in the passed reference argument the desired
	//!    behavior of the Atlas Robot simulation, as set by
	//!    set_desired_behavior().  This might not be the actual behavior the
	//!    robot is in; see set_desired_behavior() for more information.
	//!
	//! \_Parameters
	//!
	//!    \_out   desired_behavior - string reference for returned desired behavior
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_INVALID_INDEX
	//!
	AtlasErrorCode get_desired_behavior(std::string& desired_behavior);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get the current behavior of the sim.
	//!
	//! \_Description
	//!
	//!    This function returns in the passed reference argument the current
	//!    behavior of the Atlas Robot simulation.  See
	//!    set_desired_behavior() for information on why the desired and
	//!    current behaviors may not match.
	//!
	//!    This function should be called often to know when the Atlas Robot's
	//!    current behavior has changed.
	//!
	//! \_Parameters
	//!
	//!    \_out   current_behavior - string reference for returned current behavior
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!
	AtlasErrorCode get_current_behavior(std::string& current_behavior);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get the number of available behaviors.
	//!
	//! \_Description
	//!
	//!    Use with get_behavior_at_index() to enumberate the available
	//!    behaviors.
	//!
	//! \_Parameters
	//!
	//!    \_out   num_behaviors - int reference for returned number of behaviors
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!
	AtlasErrorCode get_num_behaviors(int& num_behaviors);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get the name of the behavior at a specified index.
	//!
	//! \_Description
	//!
	//!    Use with get_num_behaviors() to enumberate the available
	//!    behaviors.
	//!
	//! \_Parameters
	//!
	//!    \_in  index of behavior
	//!    \_out behavior - string reference for returned behavior name
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_INVALID_INDEX
	//!
	AtlasErrorCode get_behavior_at_index(int index, std::string& behavior);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get how much each joint is controlled by behavior calculations.
	//!
	//! \_Description
	//!
	//!    This function returns how much each joint is controlled by behavior
	//!    calculations for the specified behavior.
	//!
	//! \_Parameters
	//!
	//!    \_in   behavior              - name of behavior to check
	//!    \_out  joint_control_weights - array of weights of control by behavior
	//!
	//!    For most behaviors joints can be under only total control by
	//!    performers (return value 0.0), or total control by the behavior
	//!    (return value 1.0).
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_SUCH_BEHAVIOR
	//!
	AtlasErrorCode get_behavior_joint_weights(const std::string& behavior,
		float joint_control_weights[NUM_JOINTS]);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get how much each joint is controlled by current behavior calculations.
	//!
	//! \_Description
	//!
	//!    Similar to get_behavior_joint_weights(), but for current
	//!    behavior.
	//!
	AtlasErrorCode get_current_behavior_joint_weights(float joint_control_weights[NUM_JOINTS]);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Accessor to Atlas internal position estimates
	//!
	//! \_Description
	//!
	//!    This function will return Atlas internal odometry estimates without
	//!    having to call process_control_input().
	//!
	//! \_Parameters
	//!
	//!    \_out  robot_pos_est - internal odometry position estimate of robot base (pelvis)
	//!    \_out  foot_pos_est  - internal odometry position estimate of feet
	//!
	//!    Position estimates are with respect to an integrated odometry frame
	//!    which starts at (0, 0).  Robot base orientation is assumed to be
	//!    consistent with current IMU orientation.
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_INVALID_RESULTS
	//!
	//!    Results may be invalid if control hasn't been run yet.
	//!
	AtlasErrorCode get_estimated_position(AtlasPositionData& robot_pos_est,
		AtlasVec3f foot_pos_est[Atlas::NUM_FEET]);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get string version of error code.
	//!
	std::string get_error_code_text(AtlasErrorCode ec);


private:

	AtlasSimInterface();   //!<  Do not call directly; call create_atlas_sim_interface()
	~AtlasSimInterface();  //!<  Do not call directly; call destroy_atlas_sim_interface()

	AtlasErrorCode exercise_the_robot(const AtlasControlInput& control_input,
					  const AtlasRobotState& robot_state,
					  AtlasControlOutput& control_output);
	// Internal use only

	friend AtlasSimInterface* create_atlas_sim_interface();
	friend void destroy_atlas_sim_interface();

};

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif  // __AtlasSimInterface_H

