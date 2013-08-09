
#include <boost/thread/mutex.hpp>

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
    boost::mutex msg_lock;
    atlas_msgs::AtlasCommand last_atlas_cmd_msg;
    bool last_atlas_cmd_msg_valid;

    void atlasCommandCallback(const atlas_msgs::AtlasCommandConstPtr& msg);

  public:
    AtlasInterfaceBridge() : last_atlas_cmd_msg_valid(false) {}
    ~AtlasInterfaceBridge() {}
    // Initialize the connection to the robot.  Must be called first.
    bool start(int argc, char** argv);
    // Shut down the connection to the robot.
    bool stop();
    // Do one iteration of the interaction loop with the robot.
    void update();
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

void AtlasInterfaceBridge::update()
{
  // TODO: implement a receive/send loop that's clocked off the robot
}

void 
AtlasInterfaceBridge::atlasCommandCallback(
  const atlas_msgs::AtlasCommandConstPtr& msg)
{
  // Lock and copy the incoming command to where the main loop will pick it up
  boost::mutex::scoped_lock lock(this->msg_lock);
  ROS_INFO("Received a message");
  this->last_atlas_cmd_msg = *msg;
  this->last_atlas_cmd_msg_valid = true;
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


