/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_VRC_PLUGIN_HH
#define GAZEBO_VRC_PLUGIN_HH

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <osrf_msgs/JointCommands.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class VRCPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: VRCPlugin();

    /// \brief Destructor
    public: virtual ~VRCPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to parent world.
    /// \param[in] _sdf Pointer to sdf element.
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller on every World::Update
    private: void UpdateStates();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   List of available actions                                            //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief Sets Atlas planar navigational command velocity
    /// \param[in] _cmd A Vector3, where:
    ///   - x is the desired forward linear velocity, positive is robot-forward
    ///     and negative is robot-back.
    ///   - y is the desired lateral linear velocity, positive is robot-left
    ///     and negative is robot-right.
    ///   - z is the desired heading angular velocity, positive makes
    ///     the robot turn left, and negative makes the robot turn right
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);

    /// \brief sets robot's absolute world pose
    /// \param[in] _cmd Pose command for the robot
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);

    /// \brief sets robot's joint positions
    /// \param[in] _cmd configuration made of sensor_msgs::JointState message
    /// \todo: not yet implemented
    public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
                                       &/*_cmd*/);

    /// \brief sets robot mode via ros topic
    /// \sa SetRobotMode(const std::string &_str)
    public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    /// \param[in] _str sets robot mode by a string.  Supported modes are:
    ///  - "no_gravity" Gravity disabled for the robot.
    ///  - "nominal" Nominal "normal" physics.
    ///  - "pinned" Robot is pinned to inertial world by the pelvis.
    ///  - "feet" same as no_gravity except for r_foot and l_foot links.
    public: void SetRobotMode(const std::string &_str);


    /// \brief Robot Vehicle Interaction, put robot in driver's seat.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_pose);

    /// \brief Robot Vehicle Interaction, put robot outside driver's side door.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotExitCar(const geometry_msgs::Pose::ConstPtr &_pose);

    /// \brief Cheats to teleport fire hose to hand and make a fixed joint
    /// \param[in] _cmd Relative pose offset between the link and the hand.
    public: void RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &_cmd);

    /// \brief remove the fixed joint between robot hand link and fire hose.
    /// \param[in] _cmd not used.
    public: void RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &_cmd);


    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Generic tools for manipulating models                                //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[in] _jp joint and positions for model configuration (TODO)
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose,
                          const std::map<std::string, double> &/*_jp*/);

    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose);

    /// \brief add a constraint between 2 links
    /// \param[in] _world a pointer to the current World
    /// \param[in] _model a pointer to the Model the new Joint will be under
    /// \param[in] _link1 parent link in the new Joint
    /// \param[in] _link2 child link in the new Joint
    /// \param[in] _type string specifying joint type
    /// \param[in] _anchor a Vector3 anchor offset of the new joint
    /// \param[in] _axis Vector3 containing xyz axis of the new joint
    /// \param[in] _upper upper linit of the new joint
    /// \param[in] _lower lower linit of the new joint
    /// \return Joint created between _link1 and _link2 under _model.
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint.
    /// \param[in] _joint Joint to remove.
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief setup Robot ROS publication and sbuscriptions for the Robot
    /// These ros api describes Robot only actions
    private: void LoadRobotROSAPI();

    /// \brief setup ROS publication and sbuscriptions for VRC
    /// These ros api describes interactions between different models
    /// /atlas/cmd_vel - in pinned mode, the robot teleports based on
    ///                      messages from the cmd_vel
    private: void LoadVRCROSAPI();

    /// \brief check and spawn screw joint to simulate threads
    /// if links are aligned
    private: void CheckThreadStart();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief ROS callback queue thread
    private: void ROSQueueThread();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Atlas properties and states                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Robot
    {
      /// \brief Load the atlas portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr model;
      private: physics::LinkPtr pinLink;
      private: physics::JointPtr pinJoint;

      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
      private: math::Pose initialPose;

      /// \brief Pose of robot relative to vehicle.
      private: math::Pose vehicleRelPose;

      /// \brief Robot configuration when inside of vehicle.
      private: std::map<std::string, double> inVehicleConfiguration;

      /// \brief Flag to keep track of start-up 'harness' on the robot.
      private: bool startupHarness;

      /// \brief flag for successful initialization of atlas
      private: bool isInitialized;

      private: ros::Subscriber subTrajectory;
      private: ros::Subscriber subPose;
      private: ros::Subscriber subConfiguration;
      private: ros::Subscriber subMode;

      friend class VRCPlugin;
    } atlas;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Vehicle properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Vehicle
    {
      /// \brief Load the drc vehicle portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr model;
      private: math::Pose initialPose;
      private: physics::LinkPtr seatLink;

      /// \brief flag for successful initialization of vehicle
      private: bool isInitialized;

      friend class VRCPlugin;
    } drcVehicle;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Fire Hose (and Standpipe)                                        //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class FireHose
    {
      /// \brief set initial configuration of the fire hose link
      private: void SetInitialConfiguration()
      {
        // for (unsigned int i = 0; i < this->fireHoseJoints.size(); ++i)
        //   gzerr << "joint [" << this->fireHoseJoints[i]->GetName() << "]\n";
        // for (unsigned int i = 0; i < this->links.size(); ++i)
        //   gzerr << "link [" << this->links[i]->GetName() << "]\n";
        this->fireHoseModel->SetWorldPose(this->initialFireHosePose);
        this->fireHoseJoints[fireHoseJoints.size()-4]->SetAngle(0, -M_PI/4.0);
        this->fireHoseJoints[fireHoseJoints.size()-2]->SetAngle(0, -M_PI/4.0);
      }

      /// \brief Load the drc_fire_hose portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr fireHoseModel;
      private: physics::ModelPtr standpipeModel;

      /// joint for pinning a link to the world
      private: physics::JointPtr fixedJoint;

      /// joints and links
      private: physics::Joint_V fireHoseJoints;
      private: physics::Link_V fireHoseLinks;
      /// screw joint
      private: physics::JointPtr screwJoint;
      private: double threadPitch;

      /// Pointer to the update event connection
      private: event::ConnectionPtr updateConnection;

      private: physics::LinkPtr couplingLink;
      private: physics::LinkPtr spoutLink;
      private: math::Pose couplingRelativePose;
      private: math::Pose initialFireHosePose;

      /// \brief flag for successful initialization of fire hose, standpipe
      private: bool isInitialized;

      friend class VRCPlugin;
    } drcFireHose;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Joint Trajectory Controller                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class JointCommandsController
    {
      /// \brief Constructor, note atlas_controller is the name
      /// of the controller loaded from yaml
      private: JointCommandsController()
      {
        // initialize ros
        if (!ros::isInitialized())
        {
          gzerr << "Not loading JointCommandsController since ROS hasn't been "
                << "properly initialized.  Try starting Gazebo with"
                << " ros plugin:\n"
                << "  gazebo -s libgazebo_ros_api_plugin.so\n";
          return;
        }

        // ros stuff
        this->rosNode = new ros::NodeHandle("");

        // must match those inside AtlasPlugin
        this->jc.name.push_back("atlas::back_lbz");
        this->jc.name.push_back("atlas::back_mby");
        this->jc.name.push_back("atlas::back_ubx");
        this->jc.name.push_back("atlas::neck_ay");
        this->jc.name.push_back("atlas::l_leg_uhz");
        this->jc.name.push_back("atlas::l_leg_mhx");
        this->jc.name.push_back("atlas::l_leg_lhy");
        this->jc.name.push_back("atlas::l_leg_kny");
        this->jc.name.push_back("atlas::l_leg_uay");
        this->jc.name.push_back("atlas::l_leg_lax");
        this->jc.name.push_back("atlas::r_leg_uhz");
        this->jc.name.push_back("atlas::r_leg_mhx");
        this->jc.name.push_back("atlas::r_leg_lhy");
        this->jc.name.push_back("atlas::r_leg_kny");
        this->jc.name.push_back("atlas::r_leg_uay");
        this->jc.name.push_back("atlas::r_leg_lax");
        this->jc.name.push_back("atlas::l_arm_usy");
        this->jc.name.push_back("atlas::l_arm_shx");
        this->jc.name.push_back("atlas::l_arm_ely");
        this->jc.name.push_back("atlas::l_arm_elx");
        this->jc.name.push_back("atlas::l_arm_uwy");
        this->jc.name.push_back("atlas::l_arm_mwx");
        this->jc.name.push_back("atlas::r_arm_usy");
        this->jc.name.push_back("atlas::r_arm_shx");
        this->jc.name.push_back("atlas::r_arm_ely");
        this->jc.name.push_back("atlas::r_arm_elx");
        this->jc.name.push_back("atlas::r_arm_uwy");
        this->jc.name.push_back("atlas::r_arm_mwx");

        unsigned int n = this->jc.name.size();
        this->jc.position.resize(n);
        this->jc.velocity.resize(n);
        this->jc.effort.resize(n);
        this->jc.kp_position.resize(n);
        this->jc.ki_position.resize(n);
        this->jc.kd_position.resize(n);
        this->jc.kp_velocity.resize(n);
        this->jc.i_effort_min.resize(n);
        this->jc.i_effort_max.resize(n);

        for (unsigned int i = 0; i < n; i++)
        {
          std::vector<std::string> pieces;
          boost::split(pieces, this->jc.name[i], boost::is_any_of(":"));

          this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
            "/p", this->jc.kp_position[i]);

          this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
            "/i", this->jc.ki_position[i]);

          this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
            "/d", this->jc.kd_position[i]);

          this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
            "/i_clamp", this->jc.i_effort_min[i]);
          this->jc.i_effort_min[i] = -this->jc.i_effort_min[i];

          this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
            "/i_clamp", this->jc.i_effort_max[i]);

          this->jc.velocity[i]     = 0;
          this->jc.effort[i]       = 0;
          this->jc.kp_velocity[i]  = 0;
        }

        this->pubJointCommands =
          this->rosNode->advertise<osrf_msgs::JointCommands>(
          "/atlas/joint_commands", 1, true);

        ros::SubscribeOptions jointStatesSo =
          ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/atlas/joint_states", 1,
          boost::bind(&JointCommandsController::GetJointStates, this, _1),
          ros::VoidPtr(), this->rosNode->getCallbackQueue());
        this->subJointStates =
          this->rosNode->subscribe(jointStatesSo);
      }

      /// \brief Destructor
      private: ~JointCommandsController()
      {
        this->rosNode->shutdown();
        delete this->rosNode;
      }

      /// \brief subscriber to joint_states of the atlas robot
      private: void GetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
      {
        /// \todo: implement joint state monitoring when setting configuration
      }

      /// \brief sitting configuration of the robot when it enters
      /// the vehicle.
      /// \param[in] added pose offset when the robot is attached in the
      /// seating position.
      private: void SetSeatingConfiguration(physics::ModelPtr atlasModel)
      {
        // seated configuration
        this->jc.header.stamp = ros::Time::now();
        this->jc.position[0]  =   0.00;
        this->jc.position[1]  =   0.00;
        this->jc.position[2]  =   0.00;
        this->jc.position[3]  =   0.00;
        this->jc.position[4]  =   0.45;
        this->jc.position[5]  =   0.00;
        this->jc.position[6]  =  -1.60;
        this->jc.position[7]  =   1.60;
        this->jc.position[8]  =  -0.10;
        this->jc.position[9]  =   0.00;
        this->jc.position[10] =  -0.45;
        this->jc.position[11] =   0.00;
        this->jc.position[12] =  -1.60;
        this->jc.position[13] =   1.60;
        this->jc.position[14] =  -0.10;
        this->jc.position[15] =   0.00;
        this->jc.position[16] =   0.00;
        this->jc.position[17] =   0.00;
        this->jc.position[18] =   1.50;
        this->jc.position[19] =   1.50;
        this->jc.position[20] =  -3.00;
        this->jc.position[21] =   0.00;
        this->jc.position[22] =   0.00;
        this->jc.position[23] =   0.00;
        this->jc.position[24] =   1.50;
        this->jc.position[25] =  -1.50;
        this->jc.position[26] =  -3.00;
        this->jc.position[27] =   0.00;

        // set joint positions
        std::map<std::string, double> jps;
        for (unsigned int i = 0; i < this->jc.name.size(); ++i)
        {
          jps.insert(std::make_pair(this->jc.name[i], this->jc.position[i]));
        }
        atlasModel->SetJointPositions(jps);

        // publish JointCommands
        this->pubJointCommands.publish(jc);
      }

      /// \brief standing configuration of the robot when it exits
      /// the vehicle.
      /// \param[in] added pose offset when the robot is set down next
      /// to the vehicle.
      private: void SetStandingConfiguration(physics::ModelPtr atlasModel)
      {
        // standing configuration
        this->jc.header.stamp = ros::Time::now();
        this->jc.position[0]  =   0.00;
        this->jc.position[1]  =   0.00;
        this->jc.position[2]  =   0.00;
        this->jc.position[3]  =   0.00;
        this->jc.position[4]  =   0.00;
        this->jc.position[5]  =   0.00;
        this->jc.position[6]  =   0.00;
        this->jc.position[7]  =   0.00;
        this->jc.position[8]  =   0.00;
        this->jc.position[9]  =   0.00;
        this->jc.position[10] =   0.00;
        this->jc.position[11] =   0.00;
        this->jc.position[12] =   0.00;
        this->jc.position[13] =   0.00;
        this->jc.position[14] =   0.00;
        this->jc.position[15] =   0.00;
        this->jc.position[16] =   0.00;
        this->jc.position[17] =  -1.60;
        this->jc.position[18] =   0.00;
        this->jc.position[19] =   0.00;
        this->jc.position[20] =   0.00;
        this->jc.position[21] =   0.00;
        this->jc.position[22] =   0.00;
        this->jc.position[23] =   1.60;
        this->jc.position[24] =   0.00;
        this->jc.position[25] =   0.00;
        this->jc.position[26] =   0.00;
        this->jc.position[27] =   0.00;

        // set joint positions
        std::map<std::string, double> jps;
        for (unsigned int i = 0; i < this->jc.name.size(); ++i)
        {
          jps.insert(std::make_pair(this->jc.name[i], this->jc.position[i]));
        }
        atlasModel->SetJointPositions(jps);

        // publish JointCommands
        this->pubJointCommands.publish(jc);
      }

      /// \brief subscriber to joint_states
      private: ros::Subscriber subJointStates;

      /// \brief publisher of joint_commands
      private: ros::Publisher pubJointCommands;

      /// \brief ros node handle
      private: ros::NodeHandle* rosNode;

      /// \brief local copy of JointCommands message
      private: osrf_msgs::JointCommands jc;

      friend class VRCPlugin;
    } jointCommandsController;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: bool warpRobotWithCmdVel;
    private: double lastUpdateTime;
    private: geometry_msgs::Twist robotCmdVel;

    /// \brief fix robot butt to vehicle for efficiency
    // public: std::pair<physics::LinkPtr, physics::LinkPtr> vehicleRobot;
    public: physics::JointPtr vehicleRobotJoint;

    /// \brief Pointer to parent world.
    private: physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // default ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueueThread;

    // ros subscribers for robot actions
    private: ros::Subscriber subRobotGrab;
    private: ros::Subscriber subRobotRelease;
    private: ros::Subscriber subRobotEnterCar;
    private: ros::Subscriber subRobotExitCar;
    private: physics::JointPtr grabJoint;

    // items below are used for deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;
  };
/** \} */
/// @}
}
#endif
