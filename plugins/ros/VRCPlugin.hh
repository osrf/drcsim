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

#include <boost/thread.hpp>

#include "math/Vector3.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include "boost/thread/mutex.hpp"

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

      /// \brief sitting pose
      public: math::Pose vehiclePose;
      public: std::map<std::string, double> vehicleConfiguration;

      /// \brief fix robot butt to vehicle for efficiency
      public: std::pair<physics::LinkPtr, physics::LinkPtr> vehicleRobot;
      public: physics::JointPtr vehicleSeatJoint;

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
    } drc_vehicle;

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
    //   List of available actions                                            //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// Move the robot's pinned joint to a certain location in the world.
    private: void Teleport(const physics::LinkPtr &_pinLink,
                           physics::JointPtr &_pinJoint,
                           const math::Pose &_pose,
                           const std::map<std::string, double> &_jointPositions);

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
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr &_cmd);
    public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);
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
