
#include "atlas_ros_controller_manager.h"

int main(int argc, char** argv)
{
  AtlasRosControllerManager arcm;
  arcm.Init(argc, argv);

  arcm.Spin();

  return 0;
}
