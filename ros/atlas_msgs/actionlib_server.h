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
#include <tf/tf.h>
#include <atlas_msgs/WalkDemoAction.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasState.h>
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
  public: void ActionServerCallback();

  /// \brief Subscriber callback for BDI_interface topic
  public: void BDIStateCallback(
    const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg);

  private: void atlasStateCB(const atlas_msgs::AtlasState::ConstPtr &msg);

  private: geometry_msgs::Vector3 robotPosition;
  private: tf::Quaternion robotOrientation;

  /// \brief lock while updating control modes
  private: boost::mutex actionServerMutex;

  /// \brief ROS action server
  private: ActionServer *actionServer;

  /// \brief AtlasSimInterfaceState subscriber
  private: ros::Subscriber ASIStateSubscriber;

  /// TODO use this subscriber to get where the feet are located
  /// \brief AtlasState subscriber
  private: ros::Subscriber atlasStateSubscriber;

  /// \brief AtlasSimInterfaceCommand Publisher
  private: ros::Publisher atlasCommandPublisher;

  /// \brief local copy of the goal
  private: atlas_msgs::WalkDemoGoal activeGoal;

  /// \brief actionlib feedback
  private: atlas_msgs::WalkDemoFeedback actionServerFeedback;

  /// \brief actionlib result
  private: atlas_msgs::WalkDemoResult actionServerResult;

  /// \brief used for trajectory rollout
  private: std::vector<atlas_msgs::AtlasBehaviorStepData>
  stepTrajectory;

  /// \brief ROS node handle
  private: ros::NodeHandle rosNode;

  /// \brief bool to check if a goal is being executed, otherwise send stand
  private: bool executingGoal;

  /// \brief used to determine if a new goal exists, during which conflicting
  /// information is relayed through atlas_sim_interface_state
  private: bool newGoal;
};
