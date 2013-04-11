#ifndef ACTIONLIB_SERVER_H
#define ACTIONLIB_SERVER_H

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>

#include <atlas_msgs/AtlasSimInterfaceAction.h>
#include <actionlib/server/simple_action_server.h>

#endif // ACTIONLIB_SERVER_H

// actionlib simple action server
typedef actionlib::SimpleActionServer<atlas_msgs::AtlasSimInterfaceAction>
  ActionServer;

class ASIActionServer
{
    /// \brief constructor
    public: ASIActionServer();

    /// \brief actionlib simple action server executor callback
    private: void ActionServerCallback();

    /// \brief lock while updating control modes
    private: boost::mutex actionServerMutex;

    /// \brief actionlib simple action server
    private: ActionServer* actionServer;

    /// \brief local copy of the goal
    private: atlas_msgs::AtlasSimInterfaceGoal activeGoal;

    /// \brief actionlib feedback
    private: atlas_msgs::AtlasSimInterfaceFeedback actionServerFeedback;

    /// \brief actionlib result
    private: atlas_msgs::AtlasSimInterfaceResult actionServerResult;

    /// \brief used for trajectory rollout
    private: std::vector<atlas_msgs::AtlasBehaviorStepData>
      stepTrajectory;

    /// \brief ROS nodehandle
    private: ros::NodeHandle nh;
};
