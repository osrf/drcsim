
#ifndef __AtlasInterface_H
#define __AtlasInterface_H

#include <string>

#include "AtlasControlTypes.h"
#include "AtlasVectorTypes.h"
#include "AtlasInterfaceTypes.h"

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

using namespace Atlas;
using namespace AtlasRobot;

class AtlasInterface;


//////////////////////////////////////////////////////////////////////////////
//!
//! \mainpage  Overview
//!
//!
//!    Welcome to the Application Programming Interface (API) for the Atlas
//!    robot.
//!
//!    Click the 'Classes' or 'Files' buttons to dive into the documentation.
//!    A good place to start would be the AtlasInterface class.
//!
//!    Changes and notes <a href="../../README.html"><span>README</span></a>
//!
//!    Current version and changes to on-robot software can be accessed via
//!    network: <a href="10.66.171.30:3000/cgi-bin/atlas.cgi"><span>10.66.171.30:3000/cgi-bin/atlas.cgi</span></a>.
//!    Note that the robot must be on and on the same network as the accessing
//!    computer.
//!
//! \image html atlas.jpg
//!


//////////////////////////////////////////////////////////////////////////////
//!
//!  \class AtlasInterface AtlasInterface.h
//!
//!  \brief The primary interface for the Atlas robot.
//!
//!    This class provides an interface for communicating with the Altas
//!    robot.  There are member functions for:
//!
//!    \li   Establishing a network connection to the robot.
//!    \li   Sending control data to the robot.
//!    \li   Getting control data from the robot.
//!
//!    This class follows the singleton pattern: only one, static instance
//!    of this class is ever instantiated.  It can be created, retrieved and destroyed with
//!    with the static member functions AtlasInterface::get_instance() and AtlasInterface::delete_instance().
//!
//!    The ATLAS_ROBOT_INTERFACE environment variable must be set to the location of the AtlasRobotInterface directory in order
//!    to successfully instantiate the AtlasInterface object. This is allow for execution of the remote logging client (jrf-client)
//!    located in the tools/ subdirectory. During robot use, automatic logs may be written to the logs/ subdirectory. 
//!

class AtlasInterface
{
private:
	//////////////////////////////////////////////////////////////////////////
	//!
	//! \brief  Private constructor
	//!
	//! \_Description
	//!
	//!    Call static member get_instance() to create AtlasInterface object.
	//!
	AtlasInterface();
	//////////////////////////////////////////////////////////////////////////
	//!
	//! \brief  Private destructor
	//!
	//! \_Description
	//!
	//!    Call static member delete_instance() to destroy AtlasInterface object.
	//!
	~AtlasInterface();

public:
	//////////////////////////////////////////////////////////////////////////
	//!
	//! \brief  Create or retrieve singleton object for interfacing to the Atlas robot.
	//!
	//! \_Description
	//!
	//!    The first time this function is called an AtlasInterface object will be created.
	//!    Subsequence calls will return the same static instance.
	//!
    static AtlasErrorCode get_instance(AtlasInterface *& iface);

    //////////////////////////////////////////////////////////////////////////
	//!
	//! \brief  Destroy the AtlasInterface object.
	//!
	static AtlasErrorCode delete_instance();

//////////////////////////////////////////////////////////////////////////////
//!
//! \name Network Functions
//!
///@{

	///////////////////////////////////////////////////////
	//!
	//! \brief  Establish network connection to robot.
	//!
	//! \_Description
	//!
	//!    This function opens a network connection to the robot.  The
	//!    connection uses the UDP protocol.
	//!
	//!    No data is exchanged bewteen the robot and the caller of this
	//!    function yet; this only opens a port on the robot.
	//!
	//!    Because the UDP protocol is being used this function can succeed
	//!    even if the robot is not present or responding.  The only way to
	//!    determine whether the robot is active is to receive data from it
	//!    using the get_control_data_from_robot() function.
	//!
	//! \_Parameters
	//!
	//!    \_in   robot_ip_address - the IP address or hostname of the robot
	//!    \_in   send_port        - port on which to send data to robot
	//!    \_in   recv_port        - port on which to receive data from robot
	//!
	//!    The default values for all arguments should be used in almost all
	//!    cases.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_ROBOT_UNSPECIFIED
	//!    \li   ERROR_ROBOT_NET_CONNECT_FAILED
	//!
	AtlasErrorCode open_net_connection_to_robot(std::string robot_ip_address = "",
		int send_port = 3025,
		int recv_port = 3024);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Shut down network connection to robot.
	//!
	//! \_Description
	//!
	//!    This function closes an open network connection to the robot.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_ROBOT_NET_CONNECT_FAILED
	//!
	AtlasErrorCode close_net_connection_to_robot();


	///////////////////////////////////////////////////////
	//!
	//! \brief  Query whether net connection is open.
	//!
	//! \_Description
	//!
	//!    This function checks whether a socket connection to the robot has
	//!    been opened.  See open_net_connection_to_robot() about what this
	//!    means.
	//!
	//! \_Returns
	//!
	//!    true is net connection is open, false if closed
	//!
	bool net_connection_open();


	///////////////////////////////////////////////////////
	//!
	//! \brief  Query for robot IP address.
	//!
	//! \_Description
	//!
	//!    This function returns the robot's IP address in the passed string
	//!    reference.
	//!
	//! \_Parameters
	//!
	//!    \_in   robot_ip_address - the IP address or hostname of the robot
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!
	AtlasErrorCode get_robot_ip_address(std::string& robot_ip_address);


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Run State Functions
//!
///@{

	///////////////////////////////////////////////////////
	//!
	//! \brief  Start a robot control session.
	//!
	//! \_Description
	//!
	//!    This function will perform the necessary steps to move the robot
	//!    from RUN_STATE_IDLE, through RUN_STATE_START, to RUN_STATE_CONTROL.
	//!
	//!    If necessary this function will attempt to start the hydraulic
	//!    pump.  The function call will return immediately, though it may
	//!    take some time for the pump to start.
	//!
	//!    If no hydraulic pressure is requested, the robot should transition
	//!    to RUN_STATE_CONTROL very quickly.
	//!
	//!    During a typical control session the robot expects steady,
	//!    high-bandwidth communication from the network.  If inputs from the
	//!    send_control_data_to_robot() function are interrupted for even a
	//!    short duration the robot will automatically stop the control
	//!    session.
	//!
	//!    Note that appropriate physical switches, stops, etc., must already
	//!    be in the proper state.
	//!
	//!    This function can only be called from AtlasRobotRunState
	//!    RUN_STATE_IDLE.
	//!
	//!    start() is a command function.  No other command functions should
	//!    be called until this command completes.  The function
	//!    command_is_progress() can be called to see if another command is in
	//!    the process of running.  Under normal operation command functions
	//!    should return very quickly.
	//!
	//!  <em> Status Flags </em>
	//!
	//!    If hydraulic pressure is requested, a short time after this
	//!    function is called the AtlasRobotStatus flag STATUS_PUMP_STARTING
	//!    will be raised.
	//!
	//!    When the desired pressure is achieved the flag STATUS_PUMP_STARTING
	//!    will be lowered.
	//!
	//!  <em> State Effects </em>
	//!
	//!    A short time after this function is called the state should change
	//!    to RUN_STATE_START.
	//!
	//!    If the pump start succeeds the state will change to
	//!    AtlasRobotRunState RUN_STATE_CONTROL.
	//!
	//!    If the pump start fails the state will change to AtlasRobotRunState
	//!    RUN_STATE_STOP.
	//!
	//! \_Parameters
	//!
	//!    \_in   desired_pressure - desired hydraulic pressure output of pump
	//!    \_out  packet_seq_id    - resulting sequence id of data packet sent to robot
	//!
	//!    This command should not be considered received by the robot until
	//!    the processed_to_robot_packet_seq_id data member of
	//!    AtlasControlDataFromRobot is equal to or greater than the returned
	//!    packet_seq_id.
	//!
	//!    desired_pressure should not be set to HYDRAULIC_PRESSURE_IN_TRANSITION.
	//!    That value is used for AtlasControlDataFromRobot::pump_pressure
	//!    only.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_COMMAND_INVALID_FROM_STATE
	//!    \li   ERROR_PREV_COMMAND_IN_PROGRESS
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode start(AtlasHydraulicPressureSetting desired_pressure,
		int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Stop the current control session.
	//!
	//! \_Description
	//!
	//!    This function stops the current control session.
	//!
	//!    This includes turning off the hydraulic pump if it was on.  If the
	//!    hydraulic pump was running there may be hydraulic pressure for a
	//!    short time because the pressure does not disappear immediately.
	//!
	//!    If the stop happens while the robot is in BEHAVIOR_FREEZE the
	//!    robot will remain frozen in its current pose.
	//!
	//!    If the stop happens while the robot is in any other behavior the
	//!    robot will enter a soft freeze state, and potentially attempt to
	//!    enter a pose that reduces damage in a fall.
	//!
	//!    stop() is a command function.  See start() about limitations of
	//!    running two command functions at the same time.
	//!
	//!  <em> State Effects </em>
	//!
	//!    This function will change the state to RUN_STATE_STOP.  A short
	//!    time later the state will automatically change to
	//!    RUN_STATE_IDLE.
	//!
	//!    The robot may enter RUN_STATE_STOP automatically in some cases,
	//!    such as a critical fault.
	//!
	//!  <em> Status Flags </em>
	//!
	//!    If the stop happens while the robot is in BEHAVIOR_FREEZE, the
	//!    AtlasRobotStatus flag STATUS_SOFT_FREEZE will be set low.  In any
	//!    other behavior the STATUS_SOFT_FREEZE flag will be set high.
	//!
	//! \_Parameters
	//!
	//!    \_out  packet_seq_id - resulting sequence id of data packet sent to robot
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_COMMAND_INVALID_FROM_STATE
	//!    \li   ERROR_PREV_COMMAND_IN_PROGRESS
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode stop(int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Query whether a command function is running.
	//!
	//! \_Description
	//!
	//!    Only one command function (e.g., start(), set_desired_behavior())
	//!    can be run at a time.  No other command function should be called
	//!    while this function returns true.
	//!
	//!    Under normal operation command functions should comeplete very
	//!    quickly.
	//!
	//! \_Returns
	//!
	//!    true if a command is in progress, else false
	//!
	bool command_in_progress();


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Behavior Functions
//!
///@{

	///////////////////////////////////////////////////////
	//!
	//! \brief  Set the desired behavior of the robot.
	//!
	//! \_Description
	//!
	//!    This function will set the /desired/ behavior of the Atlas Robot.
	//!
	//!    There are a number of reasons the robot may not be able to follow
	//!    the desired behavior:
	//!
	//!    \li   robot STATUS_CONTROL_ENABLED flag is not raised (e.g., is in Soft Freeze or Idle)
	//!    \li   robot is trying to transition to a new behavior without going through Stand
	//!    \li   transition from current behavior fails for some reason
	//!
	//!    Also, it may take some time for the robot to switch from its
	//!    current behavior to the desired one as it moves through transition
	//!    motions -- e.g., decelerating from Walk to enter Stand.
	//!
	//!    The current behavior can be read from the current_behavior variable
	//!    in the AtlasControlDataFromRobot structure.
	//!
	//!    Setting the desired behavior to "User" should immediately return
	//!    all joint control to the operator, regardless of the current
	//!    behavior.
	//!
	//!    set_desired_behavior() is a command function.  See start() about
	//!    limitations of running two command functions at the same time.
	//!
	//! \_Parameters
	//!
	//!    \_in   behavior      - new desired behavior
	//!    \_out  packet_seq_id - resulting sequence id of data packet sent to robot
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_COMMAND_INVALID_FROM_STATE
	//!    \li   ERROR_PREV_COMMAND_IN_PROGRESS
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode set_desired_behavior(AtlasRobotBehavior behavior,
		int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Get the desired behavior of the robot.
	//!
	//! \_Description
	//!
	//!    This function returns in the passed reference argument the desired
	//!    behavior of the Atlas Robot, as set by set_desired_behavior().
	//!
	//!    This might not be the actual behavior the robot is in; see
	//!    set_desired_behavior() for more information.
	//!
	//! \_Parameters
	//!
	//!    \_out   desired_behavior - reference for returned desired behavior
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!
	AtlasErrorCode get_desired_behavior(AtlasRobotBehavior& desired_behavior);


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Control Data Functions
//!
///@{

	///////////////////////////////////////////////////////
	//!
	//! \brief  Send a frame of control data to the robot.
	//!
	//! \_Description
	//!
	//!    This function is for streaming control data to the robot.  The
	//!    AtlasControlDataToRobot structure should be filled out beforehand
	//!    and then passed to this function.
	//!
	//!    An internal copy of the data passed in data_to_robot is made
	//!    before this function returns.
	//!
	//! \_Parameters
	//!
	//!    \_in   data_to_robot - data to be sent to robot
	//!    \_out  packet_seq_id - resulting sequence id of sent control data packet
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_FAILED_TO_SEND_DATA
	//!    \li   ERROR_ROBOT_COMMS_TIMED_OUT
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode send_control_data_to_robot(AtlasControlDataToRobot& data_to_robot,
		int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Send a frame of extended control data to the robot.
	//!
	//! \_Description
	//!
	//!    This function is for streaming "extended" control data to the
	//!    robot.  The AtlasExtendedDataToRobot structure should be
	//!    filled out beforehand and then passed to this function.
	//!
	//!    An internal copy of the data passed in ext_data_to_robot is made
	//!    before this function returns.
	//!
	//! \_Parameters
	//!
	//!    \_in   ext_data_to_robot - data to be sent to robot
	//!    \_out  packet_seq_id     - resulting sequence id of sent control data packet
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_FAILED_TO_SEND_DATA
	//!    \li   ERROR_ROBOT_COMMS_TIMED_OUT
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode send_ext_data_to_robot(AtlasExtendedDataToRobot& ext_data_to_robot,
		int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Returns whether new control data from robot is available.
	//!
	//! \_Description
	//!
	//!    This function checks to see if new data from the robot is
	//!    available.  If so, the get_control_data_from_robot() function
	//!    should be called to retrieve the data.
	//!
	//! \_Parameters
	//!
	//!    \_in   timeout            - maximum time in seconds to wait for data
	//!    \_out  new_data_available - true if new data from the robot is available; else false
	//!
	//!    If 0.0 is passed for timeout, the function will return immediately
	//!    if no data is available.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_ROBOT_COMMS_TIMED_OUT
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode new_control_data_from_robot_available(double timeout,
		bool* new_data_available);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Retrieves most recent control data from robot.
	//!
	//! \_Description
	//!
	//!    This function retrieves the most recent set of data from the robot.
	//!
	//!    Note that there can be more than one frame of data available if the
	//!    robot is sending data faster than this function is called.  To get
	//!    all available data, call new_control_data_from_robot_available()
	//!    followed by this function until
	//!    new_control_data_from_robot_available() returns false.
	//!
	//! \_Parameters
	//!
	//!    \_out   data_from_robot - control data read from robot
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_NO_DATA_AVAILABLE
	//!    \li   ERROR_BAD_RECV_DATA
	//!    \li   ERROR_ROBOT_COMMS_TIMED_OUT
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode get_control_data_from_robot(AtlasControlDataFromRobot* data_from_robot);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Set the desired RPM of the pump.
	//!
	//! \_Description
	//!
	//!    This function sets the desired RPM (revolutions per minute) of the
	//!    hydraulic pump.
	//!
	//!    The RPM can be set from 5000 to 8000.  The default is 5000.
	//!
	//! \_Parameters
	//!
	//!    \_in   desired_pump_rpm - desired pump speed, in revolutions per minute
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_INVALID_DESIRED_PUMP_RPM
	//!
	AtlasErrorCode set_desired_pump_rpm(uint16_t desired_pump_rpm);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Downloads log file from robot.
	//!
	//! \_Description
	//!
	//!    This function retrieves a log file of data from the last run.  The
	//!    contents of the resulting log file are for Boston Dynamics use
	//!    only, for trouble-shooting problems and errors on the robot.
	//!
	//!    The environment variable ATLAS_ROBOT_INTERFACE must be set for this
	//!    function to work correctly.
	//!
	//! \_Parameters
	//!
	//!    \_in    dest_directory  - where to put files
	//!    \_in    duration        - how many seconds of log data to download
	//!    \_in    filename        - filename of logfile without filetype extension 
	//!                              (e.g. "foo") create file foo.jrf
	//!
	//!    If dest_directory is left empty, the log will be downloaded into
	//!    the default directory $ATLAS_ROBOT_INTERFACE/logs.
	//!
	//!    The filename of the log file is not supplied it will be generated 
	//!    automatically, based robot IP address or hostname, and date and time 
	//!    of download.
	//!
	//! \_Returns
	//!
	//!    Possible return values:
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_ENV_VAR_NOT_SET
	//!    \li   ERROR_FAILED_TO_RECV_DATA
	//!
	AtlasErrorCode download_robot_log_file(std::string dest_directory = "",
		float duration = 300.0f, std::string filename = "");


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Calibration Functions
//!
///@{


	///////////////////////////////////////////////////////
	//!
	//! \brief  Calibrate hardware
	//!
	//! \_Description
	//!
	//!    This function performs the following calibrations:
	//!        - servovalve null biases 
	//!	       - foot strain gauge zeros 
	//!
	//!    This is a command function.  See start() about limitations of
	//!    running two command functions at the same time.
	//!
	//!    calibrate() will implicitly change the robot's behavior to
	//!    BEHAVIOR_CALIBRATE during the calibration procedure.  The robot
	//!    will return to BEHAVIOR_FREEZE when the calibration procedure is
	//!    complete.  This function should not be called unless the robot is
	//!    in BEHAVIOR_FREEZE. This function should only be called when
	//!    the robot is hydraulically powered and hanging in the air.
	//!    If any link impacts another during the calibration procedure then 
	//!    calibration should be discarded. Re-installing robot software
	//!    will delete calibration data.
	//!
	//! \_Parameters
	//!
	//!    \_out  packet_seq_id - resulting sequence id of command
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_PREV_COMMAND_IN_PROGRESS
	//!    \li   ERROR_ILLEGAL_ARGUMENT
	//!
	AtlasErrorCode calibrate(int64_t* packet_seq_id);


	///////////////////////////////////////////////////////
	//!
	//! \brief  Force robot to perform an IMU realignment
	//!
	//!    <em> NOTE: </em> NOT IMPLEMENTED IN ROBOT SOFTWARE 1.10.0
	//!
	//! \_Description
	//!
	//!    This function forces the robot to perform an IMU realignment.
	//!
	//!    This is a command function.  See start() about limitations of
	//!    running two command functions at the same time.
	//!
	//!    Soon after this function is called the FAULT_IMU_ALIGNMENT_BAD
	//!    robot status flag will be raised.  When the operation is done
	//!    that status flag will be lowered.  The operation may take some
	//!    time, up to several minutes.
	//!
	//! \_Parameters
	//!
	//!    \_out  packet_seq_id - resulting sequence id of command
	//!
	//!    See start() RE packet_seq_id.
	//!
	//! \_Returns
	//!
	//!    \li   NO_ERRORS
	//!    \li   ERROR_NO_ROBOT_NET_CONNECTION
	//!    \li   ERROR_PREV_COMMAND_IN_PROGRESS
	//!
	AtlasErrorCode force_imu_realignment(int64_t* packet_seq_id);


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Utility Functions
//!
///@{


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for getting a string description of an error code.
	//!
	//! \_Parameters
	//!
	//!    \_in   ec - error code of interest
	//!
	//! \_Returns
	//!
	//!    string containing error code description
	//!
	std::string get_error_code_text(AtlasErrorCode ec) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for getting a string representation of an AtlasRobotRunState.
	//!
	//! \_Parameters
	//!
	//!    \_in   run_state - run state of interest
	//!
	//! \_Returns
	//!
	//!    string containing run state as text
	//!
	std::string get_run_state_as_string(AtlasRobotRunState run_state) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for getting a string representation of an AtlasRobotStatusFlag.
	//!
	//! \_Parameters
	//!
	//!    \_in   robot_status_flag - status flag of interest
	//!
	//!    Note that this should be only one status flag, not a combination
	//!    of multiple status flags.
	//!
	//! \_Returns
	//!
	//!    string containing status flag description
	//!
	std::string get_status_flag_as_string(AtlasRobotStatus robot_status_flag) const;



	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a link id enum to a string name.
	//!
	//! \_Parameters
	//!
	//!    \_in   link_id - link id enum value; e.g., LINK_LTORSO
	//!
	//! \_Returns
	//!
	//!    name of link, empty string on failure, such as for invalid id
	//!
	std::string link_name_from_link_id(AtlasLinkId link_id) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a string to a link id enum.
	//!
	//! \_Parameters
	//!
	//!    \_in   link_name - name of link; e.g., "ltorso"
	//!
	//! \_Returns
	//!
	//!    enum of link id, LINK_UNKNOWN for invalid name
	//!
	AtlasLinkId link_id_from_link_name(std::string link_name) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a joint id enum to a string name.
	//!
	//! \_Parameters
	//!
	//!    \_in   joint_id - joint id enum value; e.g., LINK_BACK_BKZ
	//!
	//! \_Returns
	//!
	//!    name of joint, empty string on failure, such as for invalid id
	//!
	std::string joint_name_from_joint_id(AtlasJointId joint_id) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a string to a joint id enum.
	//!
	//! \_Parameters
	//!
	//!    \_in   joint_name - name of link; e.g., "back_bkz"
	//!
	//! \_Returns
	//!
	//!    enum of joint, LINK_UNKNOWN for invalid name
	//!
	AtlasJointId joint_id_from_joint_name(std::string joint_name) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a behavior enum to a string name.
	//!
	//! \_Parameters
	//!
	//!    \_in   behavior - behavior enum value; e.g., BEHAVIOR_WALK
	//!
	//! \_Returns
	//!
	//!    name of behavior, empty string on failure, such as for invalid id
	//!
	std::string behavior_name_from_behavior(AtlasRobotBehavior behavior) const;


	///////////////////////////////////////////////////////
	//!
	//! \brief  Utility function for converting a string to a behavior enum.
	//!
	//! \_Parameters
	//!
	//!    \_in   behavior_name - name of link; e.g., "Walk"
	//!
	//! \_Returns
	//!
	//!    enum of behavior, BEHAVIOR_NONE for invalid name
	//!
	AtlasRobotBehavior behavior_from_behavior_name(std::string behavior_name) const;


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Removed Functions
//!
///@{

	AtlasErrorCode clear_faults(int64_t* packet_seq_id);  //!< (deprecated, now no-op)


	///@}
};


#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif  // __AtlasInterface_H

