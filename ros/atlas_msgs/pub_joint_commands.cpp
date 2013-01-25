#include <math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/common/Time.hh>
#include "sensor_msgs/JointState.h"
#include "osrf_msgs/JointCommands.h"

#include <time.h>  // for timespec


ros::CallbackQueue ros_queue_;
ros::Publisher pub_;
ros::Time last_time_;
ros::Time last_received_time_;
ros::NodeHandle* rosnode;

void queue_thread_()
{
  static const double timeout = 0.01;

  while (rosnode->ok())
  {
    ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  struct timespec tv;
  clock_gettime(0, &tv);
  gazebo::common::Time gtv = tv;

  ROS_ERROR("now [%f] received[%f] last received[%f] dt[%f]",
    gtv.Double(),
    _js->header.stamp.toSec(),
    last_received_time_.toSec(), 
    (_js->header.stamp - last_received_time_).toSec());
  last_received_time_ = _js->header.stamp;

  // if (ros::Time::now() > last_time_)
  {
    ROS_ERROR("received[%f] now[%f] dt[%f]", _js->header.stamp.toSec(),
      ros::Time::now().toSec(), (ros::Time::now() - last_time_).toSec());

    osrf_msgs::JointCommands jc;

    jc.header.stamp = _js->header.stamp;  // for testing
    // jc.header.stamp = ros::Time::now();

    jc.name.push_back("atlas::back_lbz" );
    jc.name.push_back("atlas::back_mby" );
    jc.name.push_back("atlas::back_ubx" );
    jc.name.push_back("atlas::neck_ay"  );
    jc.name.push_back("atlas::l_leg_uhz");
    jc.name.push_back("atlas::l_leg_mhx");
    jc.name.push_back("atlas::l_leg_lhy");
    jc.name.push_back("atlas::l_leg_kny");
    jc.name.push_back("atlas::l_leg_uay");
    jc.name.push_back("atlas::l_leg_lax");
    jc.name.push_back("atlas::r_leg_lax");
    jc.name.push_back("atlas::r_leg_uay");
    jc.name.push_back("atlas::r_leg_kny");
    jc.name.push_back("atlas::r_leg_lhy");
    jc.name.push_back("atlas::r_leg_mhx");
    jc.name.push_back("atlas::r_leg_uhz");
    jc.name.push_back("atlas::l_arm_elx");
    jc.name.push_back("atlas::l_arm_ely");
    jc.name.push_back("atlas::l_arm_mwx");
    jc.name.push_back("atlas::l_arm_shx");
    jc.name.push_back("atlas::l_arm_usy");
    jc.name.push_back("atlas::l_arm_uwy");
    jc.name.push_back("atlas::r_arm_elx");
    jc.name.push_back("atlas::r_arm_ely");
    jc.name.push_back("atlas::r_arm_mwx");
    jc.name.push_back("atlas::r_arm_shx");
    jc.name.push_back("atlas::r_arm_usy");
    jc.name.push_back("atlas::r_arm_uwy");

    int n = jc.name.size();
    jc.position.resize(n);
    jc.velocity.resize(n);
    jc.effort.resize(n);
    jc.kp_position.resize(n);
    jc.ki_position.resize(n);
    jc.kd_position.resize(n);
    jc.kp_velocity.resize(n);
    jc.i_effort_min.resize(n);
    jc.i_effort_max.resize(n);

    double dt = 1.0;
    double rps = 0.05;

    for (int i = 0; i < n; i++)
    {
      double theta = rps*2.0*M_PI*i*dt;
      double x1 = -0.5*sin(2*theta);
      double x2 =  0.5*sin(1*theta);

      jc.position[i]     = ros::Time::now().toSec();
      jc.velocity[i]     = 0;
      jc.effort[i]       = 0;
      jc.kp_position[i]  = 0;
      jc.ki_position[i]  = 0;
      jc.kd_position[i]  = 0;
      jc.kp_velocity[i]  = 0;
      jc.i_effort_min[i] = 0;
      jc.i_effort_max[i] = 0;
    }

    pub_.publish(jc); // use publisher
    last_time_ = ros::Time::now();
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pub_joint_trajectory_test");

  rosnode = new ros::NodeHandle();

  boost::thread callbackQueeuThread = boost::thread(queue_thread_);

  bool wait = true;
  while (wait)
  {
    last_time_ = ros::Time::now();
    if (last_time_.toSec() > 0)
      wait = false;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 20, SetJointStates,
    ros::VoidPtr(), &ros_queue_);
  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  //ros::Subscriber subJointStates = rosnode->subscribe("/atlas/joint_states", 1, SetJointStates);
 
  pub_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands",1, true);

  ros::spin();

  return 0;
}
