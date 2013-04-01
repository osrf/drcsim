
#ifndef __AtlasSimInterface_H
#define __AtlasSimInterface_H

#include <string>

#include "AtlasSimInterfaceTypes.h"

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

using namespace Atlas;


//////////////////////////////////////////////////////////////////////////////
//!
//! \mainpage  Overview
//!
//!    Click the 'Classes' or 'Files' buttons to dive into the documentation.
//!    A good place to start would be the AtlasSimInterface class.
//!
//! \image html atlas.jpg
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

}


//////////////////////////////////////////////////////////////////////////////
//!
//!  \class AtlasSimInterface AtlasSimInterface.h
//!
//!  \brief The primary interface for the DRC version of the Atlas Robot.
//!
class AtlasSimInterface
{
public:

	///////////////////////////////////////////////////////
	//!
	//! \brief  Given control inputs and current behavior, calculate control outputs.
	//!
	//! \_Description
	//!
	//!    This function will calculate control outputs that should be sent to
	//!    the Atlas Robot simulation based on values passed in in the
	//!    control_input object, and previously set behavior and related
	//!    settings.
	//!
	//!    *NOTE:* If an error is returned (e.g., the simulation is not in a
	//!    behavior), the control output will not be valid!
	//!
	//! \_Parameters
	//!
	//!    \_in   control_input   - current state of sim
	//!    \_out  control_output  - new desired state of sim gains and setpoints
	//!
	//! \_Returns
	//!
	//!    NO_ERROR on success, error code on failure
	//!
	AtlasErrorCode process_control_input(const AtlasControlDataFromRobot& control_input,
		AtlasControlDataToRobot& control_output);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Reset the behavior control.
	//!
	//! \_Description
	//!
	//!    This function should be called if:
	//!
	//!    - the simulation environment is reset
	//!    - the Atlas Robot is "teleported" to a new location
	//!    - time needs to run backward, or there's a big change in time in general
	//!
	//!    The next control data from the robot sim after reset will be taken
	//!    as the initial conditions for a new simulation run.
	//!
	//! \_Returns
	//!
	//!    NO_ERROR on success, error code on failure
	//!
	AtlasErrorCode reset_control();


	///////////////////////////////////////////////////////
	//!
	//! \brief  Set the desired behavior of the sim.
	//!
	//! \_Description
	//!
	//!    This function will set the *desired* behavior of the Atlas
	//!    Robot simulation.
	//!
	//!    There are a number of reasons the simulation may not be able to
	//!    follow the desired behavior.  For example, most desired behaviors
	//!    will not be achievable if the robot has fallen.  Also, it may take
	//!    a little while for the robot to switch from its current behavior
	//!    to the desired one.
	//!
	//!    Setting the desired behavior to "none" should immediately return
	//!    all joint control to the performer.
	//!
	//!    Current supported behaviors should include:
	//!
	//!      - "none"  - no control from behavior; all values set by performer
	//!      - "stand" - simple stand behavior; some values set by behavior
	//!      - "walk"  - simple walk behavior; some values set by behavior
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
	//!    NO_ERROR on success, error code on failure
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
	//!    NO_ERROR on success, error code on failure
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
	//!    NO_ERROR on success, error code on failure
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
	//!    NO_ERROR on success, error code on failure
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
	//!    NO_ERROR on success, error code on failure
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
	//!    \_in   behavior         - name of behavior to check
	//!    \_out  joint_control_weights - array of weights of control by behavior
	//!
	//!    Currently joints can be under only total control by performers
	//!    (return value 0.0), or total control by the behavior
	//!    (return value 1.0).
	//!
	//! \_Returns
	//!
	//!    NO_ERROR on success, error code on failure
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
	//! \brief  Get string version of error code.
	//!
	std::string get_error_code_text(AtlasErrorCode ec);


	//////////////////////////////////////////////////////////////////////////////
	//!
	//! \name Parameter Functions
	//!
	//!    These functions are used to set some behavior parameters. The
	//!    parameters and their meanings depend on the behavior.
	///@{

	AtlasErrorCode set_behavior_parameter_1f(AtlasBehaviorParameterId parameter_id,
		float v);

	AtlasErrorCode get_behavior_parameter_1f(AtlasBehaviorParameterId parameter_id,
		float& v);

	AtlasErrorCode set_behavior_parameter_2f(AtlasBehaviorParameterId parameter_id,
		float v0,
		float v1);

	AtlasErrorCode get_behavior_parameter_2f(AtlasBehaviorParameterId parameter_id,
		float& v0,
		float& v1);

	AtlasErrorCode set_behavior_parameter_3f(AtlasBehaviorParameterId parameter_id,
		float v0,
		float v1,
		float v2);

	AtlasErrorCode get_behavior_parameter_3f(AtlasBehaviorParameterId parameter_id,
		float& v0,
		float& v1,
		float& v2);

	AtlasErrorCode set_behavior_parameter_quat(AtlasBehaviorParameterId parameter_id,
		const AtlasQuaternion& v);

	AtlasErrorCode get_behavior_parameter_quat(AtlasBehaviorParameterId parameter_id,
		AtlasQuaternion& v);

	///@}

private:

	AtlasSimInterface();   //!<  Do not call directly; call create_atlas_sim_interface()
	~AtlasSimInterface();  //!<  Do not call directly; call destroy_atlas_sim_interface()

	friend AtlasSimInterface* create_atlas_sim_interface();
	friend void destroy_atlas_sim_interface();

};

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif  // __AtlasSimInterface_H

