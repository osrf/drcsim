
#include <ros/ros.h>
#include <AtlasInterface.h>
#include <AtlasInterfaceTypes.h>

class AtlasInterfaceBridge
{
  private:
    AtlasInterface the_atlas;
    //AtlasUtility u_atlas

  public:
    AtlasInterfaceBridge() {}
    ~AtlasInterfaceBridge() {}
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_interface_bridge");

  AtlasInterfaceBridge aib;
/*

  //
  //- Start up the Atlas robot.  This call will take the robot through the
  //  various run states until it reaches RUN_STATE_CONTROL, or fails to do
  //  so for some reason.
  //
  //  (Function in AtlasUtility.cpp)
  //
  bool start_succeeded = u_atlas->start_control_session(argc, argv);

  if (!start_succeeded)
  {
  	printf(PROGRAM_NAME " ERROR: Failed to start Atlas robot.\n");

  	delete the_atlas;
  	delete u_atlas;

  	exit(-1);
  }

  //
  //- Control the robot.  This call will return when the robot is stopping
  //  for some reason, and the run state is leaving RUN_STATE_CONTROL.
  //
  //  (Functions in [controller_name]_controller.cpp)
  //
  bool control_succeeded = false;

  if ((u_atlas->m_controller == "") || (u_atlas->m_controller == "simple"))
  {
  	control_succeeded = simple_controller();
  }
  else if (u_atlas->m_controller == "behavior")
  {
  	control_succeeded = behavior_controller();
  }
  else if (u_atlas->m_controller == "force_control")
  {
  	control_succeeded = force_control_controller();
  }
  else
  {
  	fprintf(stderr, PROGRAM_NAME " ERROR: Unknown controller name '%s'.\n",
  		u_atlas->m_controller.c_str());
  	control_succeeded = false;
  }

  if (!control_succeeded)
  {
  	fprintf(stderr, PROGRAM_NAME " ERROR: Error while controlling Atlas robot.\n");
  }

  //
  //- Stop the control session.  This call will return when the robot has
  //  entered RUN_STATE_IDLE.
  //
  //  (Function in AtlasUtility.cpp)
  //
  bool stop_succeeded = u_atlas->stop_control_session();

  if (!stop_succeeded)
  {
  	fprintf(stderr, PROGRAM_NAME " ERROR: Error while stopping Atlas robot.\n");
  }

  delete the_atlas;
  delete u_atlas;
*/

  return 0;
}


