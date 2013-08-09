
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

  return 0;
}


