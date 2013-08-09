
#include <ros/ros.h>
#include <AtlasInterface.h>
#include <AtlasInterfaceTypes.h>
#include <AtlasUtility.h>

#include <atlas_msgs/AtlasCommand.h>

class AtlasInterfaceBridge
{
  private:
    AtlasInterface the_atlas;
    AtlasUtility u_atlas;
    ros::Subscriber atlas_cmd_sub;
    ros::NodeHandle node;

  public:
    AtlasInterfaceBridge() {}
    ~AtlasInterfaceBridge() {}
    bool start(int argc, char** argv);
    bool stop();
    void atlasCommandCallback(const atlas_msgs::AtlasCommandConstPtr& msg);
};

bool AtlasInterfaceBridge::start(int argc, char** argv)
{
  bool result = this->u_atlas.start_control_session(argc, argv);
  this->atlas_cmd_sub = this->node.subscribe("atlas/atlas_command", 1,
    &AtlasInterfaceBridge::atlasCommandCallback, this);
  return result;
}

bool AtlasInterfaceBridge::stop()
{
  this->atlas_cmd_sub.shutdown();
  bool result = this->u_atlas.stop_control_session();
  return result;
}

void 
AtlasInterfaceBridge::atlasCommandCallback(
  const atlas_msgs::AtlasCommandConstPtr& msg)
{
  ROS_INFO("Received a message");
  // TODO: stuff an instance of AtlasControlDataToRobot and send it.
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_interface_bridge");

  AtlasInterfaceBridge aib;

  if(!aib.start(argc, argv))
  {
    ROS_ERROR("Failed to start Atlas robot.");
    exit(1);
  }

  if(!aib.stop())
  {
    ROS_ERROR("Failed to stop Atlas robot.");
    exit(1);
  }

  return 0;
}


