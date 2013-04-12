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

#include <atlas_msgs/WalkDemoAction.h>
#include <actionlib/server/simple_action_server.h>

#define NUM_REQUIRED_WALK_STEPS 4

#endif // ACTIONLIB_SERVER_H

// actionlib simple action server
typedef actionlib::SimpleActionServer<atlas_msgs::WalkDemoAction> ActionServer;



class ASIActionServer
{
  /// \brief constructor
  public: ASIActionServer();

  /// \brief actionlib simple action server executor callback
  private: void ActionServerCallback();

  /// \brief Subscriber callback for BDI_interface topic
  private: void BDIStateCallback();

  /// \brief lock while updating control modes
  private: boost::mutex actionServerMutex;

  /// \brief actionlib simple action server
  private: ActionServer* actionServer;

  private: AtlasStateSubscriber* atlasStateSubscriber;

  private: ros::Publisher commandPublisher;

  /// \brief local copy of the goal
  private: atlas_msgs::WalkDemoGoal activeGoal;

  /// \brief actionlib feedback
  private: atlas_msgs::WalkDemoFeedback actionServerFeedback;

  /// \brief actionlib result
  private: atlas_msgs::WalkDemoResult actionServerResult;

  /// \brief used for trajectory rollout
  private: std::vector<atlas_msgs::AtlasBehaviorStepData>
  stepTrajectory;

  private: atlas_msgs::AtlasSimInterfaceFeedback asiFeedback;
  private: bool executingGoal;

  private: AtlasSimInterface *atlasSimInterface;

  /// \brief ROS nodehandle
  private: ros::NodeHandle rosNode;
};
