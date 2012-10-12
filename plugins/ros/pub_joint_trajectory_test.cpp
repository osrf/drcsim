#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo/math/Quaternion.hh>
#include <math.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pub_joint_trajectory_test");
  ros::NodeHandle rosnode;
  ros::Publisher pub_ = rosnode.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory",100);

  trajectory_msgs::JointTrajectory jt;

  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "drc_robot::r_foot";

  jt.joint_names.push_back("drc_robot::back.lbz" );
  jt.joint_names.push_back("drc_robot::back.mby" );
  jt.joint_names.push_back("drc_robot::back.ubx" );
  jt.joint_names.push_back("drc_robot::neck.ay"  );
  jt.joint_names.push_back("drc_robot::l.leg.uhz");
  jt.joint_names.push_back("drc_robot::l.leg.mhx");
  jt.joint_names.push_back("drc_robot::l.leg.lhy");
  jt.joint_names.push_back("drc_robot::l.leg.kny");
  jt.joint_names.push_back("drc_robot::l.leg.uay");
  jt.joint_names.push_back("drc_robot::l.leg.lax");
  jt.joint_names.push_back("drc_robot::r.leg.lax");
  jt.joint_names.push_back("drc_robot::r.leg.uay");
  jt.joint_names.push_back("drc_robot::r.leg.kny");
  jt.joint_names.push_back("drc_robot::r.leg.lhy");
  jt.joint_names.push_back("drc_robot::r.leg.mhx");
  jt.joint_names.push_back("drc_robot::r.leg.uhz");
  jt.joint_names.push_back("drc_robot::l.arm.elx");
  jt.joint_names.push_back("drc_robot::l.arm.ely");
  jt.joint_names.push_back("drc_robot::l.arm.mwx");
  jt.joint_names.push_back("drc_robot::l.arm.shx");
  jt.joint_names.push_back("drc_robot::l.arm.usy");
  jt.joint_names.push_back("drc_robot::l.arm.uwy");
  jt.joint_names.push_back("drc_robot::r.arm.elx");
  jt.joint_names.push_back("drc_robot::r.arm.ely");
  jt.joint_names.push_back("drc_robot::r.arm.mwx");
  jt.joint_names.push_back("drc_robot::r.arm.shx");
  jt.joint_names.push_back("drc_robot::r.arm.usy");
  jt.joint_names.push_back("drc_robot::r.arm.uwy");

  int n = 500;
  double dt = 0.1;
  double rps = 0.05;
  jt.points.resize(n);
  for (int i = 0; i < n; i++)
  {
    double theta = rps*2.0*M_PI*i*dt;
    double x1 = -0.5*sin(2*theta);
    double x2 =  0.5*sin(1*theta);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x1);

    // set duration
    jt.points[i].time_from_start = ros::Duration(dt);
    ROS_INFO("test: angles[%d][%f, %f]",n,x1,x2);
  }

  while(true)
  {
    pub_.publish(jt); // use publisher
    sleep(10);
  }

  return 0;
}
