/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_VRC_PLUGIN_HH
#define GAZEBO_VRC_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread.hpp>

#include "math/Vector3.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include "boost/thread/mutex.hpp"

typedef actionlib::SimpleActionClient<
  control_msgs::FollowJointTrajectoryAction > TrajClient;
namespace gazebo
{
  class VRCPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: VRCPlugin();

    /// \brief Destructor
    public: virtual ~VRCPlugin();

    /// \brief Load the controller
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world;

    /// \brief Mutex for VRC Plugin
    private: boost::mutex update_mutex;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Robot properties and states                                      //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Robot
    {
      public: physics::ModelPtr model;
      public: physics::LinkPtr pinLink;
      public: physics::JointPtr pinJoint;

      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
      public: math::Pose initialPose;

      /// \brief Pose of robot relative to vehicle
      public: math::Pose vehicleRelPose;

      /// \brief robot configuration when inside of vehicle
      public: std::map<std::string, double> inVehicleConfiguration;

      /// flag to keep track of start-up 'harness' on the robot
      public: bool startupHarness;

      public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      public: ros::Subscriber trajectory_sub_;
      public: ros::Subscriber pose_sub_;
      public: ros::Subscriber configuration_sub_;
      public: ros::Subscriber mode_sub_;

    } drc_robot;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Vehicle properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Vehicle
    {
      public: physics::ModelPtr model;
      public: math::Pose initialPose;
      public: physics::LinkPtr seatLink;

      public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    } drc_vehicle;


    /// \brief fix robot butt to vehicle for efficiency
    // public: std::pair<physics::LinkPtr, physics::LinkPtr> vehicleRobot;
    public: physics::JointPtr vehicleRobotJoint;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Fire Hose   properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: physics::ModelPtr fire_hose;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Standpipe   properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: physics::ModelPtr standpipe;


    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Joint Trajectory Controller                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////


    private: class JointTrajectory
    {
    public:
      // Action client for the joint trajectory action 
      // used to trigger the arm movement action
      TrajClient* traj_client_;

    public:
      //! Initialize the action client and wait for action server to come up
      JointTrajectory() 
      {
        // tell the action client that we want to spin a thread by default
        //traj_client_ = new TrajClient("/drc_controller/joint_trajectory_action", true);
        traj_client_ = new TrajClient("/drc_controller/follow_joint_trajectory", true);

      }

      //! Clean up the action client
      ~JointTrajectory()
      {
        delete traj_client_;
      }

      //! Sends the command to start a given trajectory
      void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
      {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

        traj_client_->sendGoal(goal);
      }

      //! Generates a simple trajectory with two waypoints, used as an example
      /*! Note that this trajectory contains two waypoints, joined together
          as a single trajectory. Alternatively, each of these waypoints could
          be in its own trajectory - a trajectory can have one or more waypoints
          depending on the desired application.
      */
      control_msgs::FollowJointTrajectoryGoal seatingConfiguration()
      {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("l_leg_uhz");
        goal.trajectory.joint_names.push_back("l_leg_mhx");
        goal.trajectory.joint_names.push_back("l_leg_lhy");
        goal.trajectory.joint_names.push_back("l_leg_kny");
        goal.trajectory.joint_names.push_back("l_leg_uay");
        goal.trajectory.joint_names.push_back("l_leg_lax");

        goal.trajectory.joint_names.push_back("r_leg_uhz");
        goal.trajectory.joint_names.push_back("r_leg_mhx");
        goal.trajectory.joint_names.push_back("r_leg_lhy");
        goal.trajectory.joint_names.push_back("r_leg_kny");
        goal.trajectory.joint_names.push_back("r_leg_uay");
        goal.trajectory.joint_names.push_back("r_leg_lax");

        goal.trajectory.joint_names.push_back("l_arm_usy");
        goal.trajectory.joint_names.push_back("l_arm_shx");
        goal.trajectory.joint_names.push_back("l_arm_ely");
        goal.trajectory.joint_names.push_back("l_arm_elx");
        goal.trajectory.joint_names.push_back("l_arm_uwy");
        goal.trajectory.joint_names.push_back("l_arm_mwx");

        goal.trajectory.joint_names.push_back("r_arm_usy");
        goal.trajectory.joint_names.push_back("r_arm_shx");
        goal.trajectory.joint_names.push_back("r_arm_ely");
        goal.trajectory.joint_names.push_back("r_arm_elx");
        goal.trajectory.joint_names.push_back("r_arm_uwy");
        goal.trajectory.joint_names.push_back("r_arm_mwx");

        goal.trajectory.joint_names.push_back("neck_ay"  );
        goal.trajectory.joint_names.push_back("back_lbz" );
        goal.trajectory.joint_names.push_back("back_mby" );
        goal.trajectory.joint_names.push_back("back_ubx" );

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(28);
        goal.trajectory.points[ind].positions[0]  =   0.00;
        goal.trajectory.points[ind].positions[1]  =   0.00;
        goal.trajectory.points[ind].positions[2]  =  -1.70;
        goal.trajectory.points[ind].positions[3]  =   1.80;
        goal.trajectory.points[ind].positions[4]  =  -0.10;
        goal.trajectory.points[ind].positions[5]  =   0.00;

        goal.trajectory.points[ind].positions[6]  =   0.00;
        goal.trajectory.points[ind].positions[7]  =   0.00;
        goal.trajectory.points[ind].positions[8]  =  -1.70;
        goal.trajectory.points[ind].positions[9]  =   1.80;
        goal.trajectory.points[ind].positions[10] =  -0.10;
        goal.trajectory.points[ind].positions[11] =   0.00;

        goal.trajectory.points[ind].positions[12] =  -1.60;
        goal.trajectory.points[ind].positions[13] =  -1.60;
        goal.trajectory.points[ind].positions[14] =   0.00;
        goal.trajectory.points[ind].positions[15] =   0.00;
        goal.trajectory.points[ind].positions[16] =   0.00;
        goal.trajectory.points[ind].positions[17] =   0.00;

        goal.trajectory.points[ind].positions[18] =   1.60;
        goal.trajectory.points[ind].positions[19] =   1.60;
        goal.trajectory.points[ind].positions[20] =   0.00;
        goal.trajectory.points[ind].positions[21] =   0.00;
        goal.trajectory.points[ind].positions[22] =   0.00;
        goal.trajectory.points[ind].positions[23] =   0.00;

        goal.trajectory.points[ind].positions[24] =   0.00;
        goal.trajectory.points[ind].positions[25] =   0.00;
        goal.trajectory.points[ind].positions[26] =   0.00;
        goal.trajectory.points[ind].positions[27] =   0.00;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }

        // tolerances
        /*
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 0.1;
          jt.velocity = 0.1;
          jt.acceleration = 0.1;
          goal.path_tolerance.push_back(jt);
        }
        */

        // tolerances
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 1000;
          jt.velocity = 1000;
          jt.acceleration = 1000;
          goal.goal_tolerance.push_back(jt);
        }

        goal.goal_time_tolerance.sec = 10;
        goal.goal_time_tolerance.nsec = 0;

        //we are done; return the goal
        return goal;
      }

      control_msgs::FollowJointTrajectoryGoal standingConfiguration()
      {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("l_leg_uhz");
        goal.trajectory.joint_names.push_back("l_leg_mhx");
        goal.trajectory.joint_names.push_back("l_leg_lhy");
        goal.trajectory.joint_names.push_back("l_leg_kny");
        goal.trajectory.joint_names.push_back("l_leg_uay");
        goal.trajectory.joint_names.push_back("l_leg_lax");

        goal.trajectory.joint_names.push_back("r_leg_uhz");
        goal.trajectory.joint_names.push_back("r_leg_mhx");
        goal.trajectory.joint_names.push_back("r_leg_lhy");
        goal.trajectory.joint_names.push_back("r_leg_kny");
        goal.trajectory.joint_names.push_back("r_leg_uay");
        goal.trajectory.joint_names.push_back("r_leg_lax");

        goal.trajectory.joint_names.push_back("l_arm_usy");
        goal.trajectory.joint_names.push_back("l_arm_shx");
        goal.trajectory.joint_names.push_back("l_arm_ely");
        goal.trajectory.joint_names.push_back("l_arm_elx");
        goal.trajectory.joint_names.push_back("l_arm_uwy");
        goal.trajectory.joint_names.push_back("l_arm_mwx");

        goal.trajectory.joint_names.push_back("r_arm_usy");
        goal.trajectory.joint_names.push_back("r_arm_shx");
        goal.trajectory.joint_names.push_back("r_arm_ely");
        goal.trajectory.joint_names.push_back("r_arm_elx");
        goal.trajectory.joint_names.push_back("r_arm_uwy");
        goal.trajectory.joint_names.push_back("r_arm_mwx");

        goal.trajectory.joint_names.push_back("neck_ay"  );
        goal.trajectory.joint_names.push_back("back_lbz" );
        goal.trajectory.joint_names.push_back("back_mby" );
        goal.trajectory.joint_names.push_back("back_ubx" );

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(28);
        goal.trajectory.points[ind].positions[0]  =   0.00;
        goal.trajectory.points[ind].positions[1]  =   0.00;
        goal.trajectory.points[ind].positions[2]  =   0.00;
        goal.trajectory.points[ind].positions[3]  =   0.00;
        goal.trajectory.points[ind].positions[4]  =   0.00;
        goal.trajectory.points[ind].positions[5]  =   0.00;

        goal.trajectory.points[ind].positions[6]  =   0.00;
        goal.trajectory.points[ind].positions[7]  =   0.00;
        goal.trajectory.points[ind].positions[8]  =   0.00;
        goal.trajectory.points[ind].positions[9]  =   0.00;
        goal.trajectory.points[ind].positions[10] =   0.00;
        goal.trajectory.points[ind].positions[11] =   0.00;

        goal.trajectory.points[ind].positions[12] =   0.00;
        goal.trajectory.points[ind].positions[13] =  -1.60;
        goal.trajectory.points[ind].positions[14] =   0.00;
        goal.trajectory.points[ind].positions[15] =   0.00;
        goal.trajectory.points[ind].positions[16] =   0.00;
        goal.trajectory.points[ind].positions[17] =   0.00;

        goal.trajectory.points[ind].positions[18] =   0.00;
        goal.trajectory.points[ind].positions[19] =   1.60;
        goal.trajectory.points[ind].positions[20] =   0.00;
        goal.trajectory.points[ind].positions[21] =   0.00;
        goal.trajectory.points[ind].positions[22] =   0.00;
        goal.trajectory.points[ind].positions[23] =   0.00;

        goal.trajectory.points[ind].positions[24] =   0.00;
        goal.trajectory.points[ind].positions[25] =   0.00;
        goal.trajectory.points[ind].positions[26] =   0.00;
        goal.trajectory.points[ind].positions[27] =   0.00;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }

        // tolerances
        /*
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 0.1;
          jt.velocity = 0.1;
          jt.acceleration = 0.1;
          goal.path_tolerance.push_back(jt);
        }
        */

        // tolerances
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 1000;
          jt.velocity = 1000;
          jt.acceleration = 1000;
          goal.goal_tolerance.push_back(jt);
        }

        goal.goal_time_tolerance.sec = 10;
        goal.goal_time_tolerance.nsec = 0;

        //we are done; return the goal
        return goal;
      }

      //! Returns the current state of the action
      actionlib::SimpleClientGoalState getState()
      {
        return traj_client_->getState();
      }
     
    } joint_trajectory_controller;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   List of available actions                                            //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// Move the robot's pinned joint to a certain location in the world.
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose,
                          const std::map<std::string, double> &_jointPositions);

    /// \brief sets robot's absolute world pose
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose)
      {
        // default empty joint positions if not provided
        std::map<std::string, double> jointPositions;
        this->Teleport(_pinLink, _pinJoint, _pose, jointPositions);
      }

    /// Sets DRC Robot planar navigational command velocity
    /// _cmd is a Vector3, where:
    ///   - x is the desired forward linear velocity, positive is robot-forward
    ///     and negative is robot-back.
    ///   - y is the desired lateral linear velocity, positive is robot-left
    ///     and negative is robot-right.
    ///   - z is the desired heading angular velocity, positive makes
    ///     the robot turn left, and negative makes the robot turn right
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);

    /// \brief sets robot's absolute world pose
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);

    /// \brief sets robot's joint positions
    public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
                                       &_cmd);

    /// \brief sets robot mode via ros topic
    public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    public: void SetRobotMode(const std::string &_str);


    // \brief Robot Vehicle Interaction
    public: void RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void RobotExitCar(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void SetHandWheelPose(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void SetGasPedalPose(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void SetBrakePedalPose(const geometry_msgs::Pose::ConstPtr &_cmd);

    // \brief Cheats to teleport fire hose to hand and make a fixed joint
    public: void RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &_cmd);

    // \brief create a fixed joint between robot hand link and a nearby link
    public: void RobotGrabLink(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &_cmd);



    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Generic tools for manipulating models                                //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief add a constraint between 2 links
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief setup Robot ROS publication and sbuscriptions for the Robot
    /// These ros api describes Robot only actions
    private: void LoadRobotROSAPI();

    /// \brief setup ROS publication and sbuscriptions for VRC
    /// These ros api describes interactions between different models
    /// /drc_robot/cmd_vel - in pinned mode, the robot teleports based on
    ///                      messages from the cmd_vel
    private: void LoadVRCROSAPI();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    // private: std::map<physics::LinkPtr, physics::JointPtr> pins;

    private: bool warpRobotWithCmdVel;

    private: double lastUpdateTime;
    private: geometry_msgs::Twist robotCmdVel;

    // ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue ros_queue_;
    private: void ROSQueueThread();
    private: boost::thread callback_queue_thread_;

    // ros subscription for grabbing objects
    public: ros::Subscriber robot_grab_sub_;
    public: ros::Subscriber robot_release_sub_;
    public: ros::Subscriber robot_enter_car_sub_;
    public: ros::Subscriber robot_exit_car_sub_;
    private: physics::JointPtr grabJoint;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
  };
/** \} */
/// @}
}
#endif
