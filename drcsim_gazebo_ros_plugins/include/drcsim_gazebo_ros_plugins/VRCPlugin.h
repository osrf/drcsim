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

#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>

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
    /// \param[in] _duration If > 0.0 stop applying the commanded
    ///                      velocity after the specific duration, in seconds.
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd,
                                double _duration);

    /// \brief Calls through to SetRobotCmdVel with a _duration of 0.0.
    ///        Used as a ROS message callback.
    public: void SetRobotCmdVelTopic(
      const geometry_msgs::Twist::ConstPtr &_cmd);

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

    /// \brief Accepts BDI behavior library commands and fakes them
    /// \param[in] _asic the incoming command
    public: void SetFakeASIC(
      const atlas_msgs::AtlasSimInterfaceCommand::ConstPtr &_asic);

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
                                        double _upper, double _lower,
                                        bool _disableCollision = false);

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

    /// \brief Helper for pinning Atlas to the world.
    /// \param[in] _with_gravity Whether to enable gravity on the robot's
    /// links after pinning it.
    private: void PinAtlas(bool _with_gravity);

    /// \brief Helper for unpinning Atlas to the world.
    private: void UnpinAtlas();

    /// \brief Helper for disabling foot collisions
    /// \param[in] _mode collision mode; will be passed to
    ///   gazebo::physics::Link::SetCollideMode()
    private: void SetFeetCollide(const std::string &_mode);

    /// \brief Helper to convert step data to a planar cmd_vel-style Twist
    /// \param[in] _step the last step to be taken
    /// \param[in] _dt the desired duration until _step is reached
    /// \param[out] _twist destination to write the cmd_vel data
    private: void StepDataToTwist(
               const atlas_msgs::AtlasBehaviorStepData & _step,
               double _dt,
               geometry_msgs::Twist::Ptr _twist);

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Atlas properties and states                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Robot
    {
      public:
        Robot();
        ~Robot();

      /// \brief Load the atlas portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void InsertModel(physics::WorldPtr _parent,
        sdf::ElementPtr _sdf);

      /// \brief Spawns a gazebo robot model from string.
      /// \param[in] _robotStr string containing model sdf or urdf.
      /// \param[in] _modelName name of newly spawned model in gazebo.
      /// \param[in] _spawnPose spawn location of model in world frame.
      /// \return pointer to the newly spawned model.
      private: bool CheckGetModel(physics::WorldPtr _world);

      private: math::Pose spawnPose;
      private: physics::ModelPtr model;
      private: physics::LinkPtr pinLink;
      private: physics::JointPtr pinJoint;

      private: std::string modelName;
      private: std::string pinLinkName;

      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
      private: math::Pose initialPose;

      /// \brief Pose of robot relative to vehicle.
      private: math::Pose vehicleRelPose;

      /// \brief Robot configuration when inside of vehicle.
      private: std::map<std::string, double> inVehicleConfiguration;

      /// \brief  At t-t0 < startupStandPrepDuration seconds, pinned.
      /// At t - t0 = startupStandPrepDurationl, start StandPrep mode.
      private: double startupStandPrepDuration;

      /// \brief at t - t0 = startupNominal, start Nominal mode.
      private: double startupNominal;

      /// \brief at t - t0 = startupStand, start Stand mode.
      private: double startupStand;

      /// \brief Flag to keep track of start-up 'bdi_stand' on the robot.
      private: enum BDIStandSequence {
        BS_NONE = 0,
        BS_PID_PINNED = 1,
        BS_STAND_PREP_PINNED = 2,
        BS_STAND_PREP = 3,
        BS_INITIALIZED = 4
      };
      private: int bdiStandSequence;

      /// \brief Flag to keep track of start-up 'pinned' mode
      private: enum PinnedSequence {
        PS_NONE = 0,
        PS_PINNED = 1,
        PS_INITIALIZED = 2
      };
      private: int pinnedSequence;

      // mode flag to indicate nominal mode has already been called once.
      private: bool bdiStandNominal;

      /// \brief Keep track of start-up 'bdi_stand' time
      private: common::Time startupBDIStandStartTime;

      /// \brief allow user to set startup mode as bdi_stand or pinned
      private: std::string startupMode;

      /// \brief flag for successful initialization of atlas
      private: enum StartupSequence {
        NONE = 0,
        SPAWN_QUEUED = 1,
        SPAWN_SUCCESS = 2,
        INIT_MODEL_SUCCESS = 3,
        INITIALIZED = 4
      };
      private: int startupSequence;

      private: double startupHarnessDuration;

      private: ros::Subscriber subTrajectory;
      private: ros::Subscriber subPose;
      private: ros::Subscriber subConfiguration;
      private: ros::Subscriber subMode;
      private: ros::Subscriber subFakeASIC;
      /// \brief publisher of fake AtlasSimInterfaceState
      private: ros::Publisher pubFakeASIS;
      /// \brief current requested (fake) behavior
      private: int currentBehavior;
      /// \brief current (fake) step being pursued
      private: int currentStepIndex;
      /// \brief last (fake) step in the current sequence
      private: int lastStepIndex;

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
      private: void SetInitialConfiguration();

      /// \brief Load the drc_fire_hose portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr fireHoseModel;
      private: physics::ModelPtr standpipeModel;
      private: physics::ModelPtr valveModel;
      private: physics::JointPtr valveJoint;

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
    //   Robot Joint Controller                                               //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class AtlasCommandController
    {
      /// \brief Constructor, note atlas_controller is the name
      /// of the controller loaded from yaml
      private: AtlasCommandController();

      /// \brief Destructor
      private: ~AtlasCommandController();

      /// \brief: initialize AtlasCommandController with atlas model pointer
      /// \param[in] Atlas model pointer
      private: void InitModel(physics::ModelPtr _model);

      /// \brief: atlas model pointer
      private: physics::ModelPtr model;

      /// \brief Checks atlas model for joint names
      /// used to find joint name since atlas_v3 remapped some joint names
      /// \param[in] possible joint name
      /// \param[in] possible joint name
      /// \return _st1 or _st2 whichever is a valid joint, else empty str.
      private: std::string FindJoint(std::string _st1, std::string _st2);
      private: std::string FindJoint(std::string _st1, std::string _st2, std::string _st3);

      /// \brief subscriber to joint_states of the atlas robot
      private: void GetJointStates(
        const sensor_msgs::JointState::ConstPtr &_js);

      /// \brief stand configuration with PID controller
      /// \param[in] pointer to atlas model
      private: void SetPIDStand(physics::ModelPtr atlasModel);

      /// \brief switch to Freeze Mode
      private: void SetBDIFREEZE();

      /// \brief switch to StandPrep Mode
      private: void SetBDIStandPrep();

      /// \brief switch to Stand Mode
      private: void SetBDIStand();

      /// \brief sitting configuration of the robot when it enters the vehicle.
      /// \param[in] pointer to atlas model
      private: void SetSeatingConfiguration(physics::ModelPtr atlasModel);

      /// \brief standing configuration of the robot when it exits the vehicle.
      /// \param[in] _atlasModel pointer to atlas model
      private: void SetStandingConfiguration(physics::ModelPtr _atlasModel);

      /// \brief subscriber to joint_states
      private: ros::Subscriber subJointStates;

      /// \brief publisher of joint_commands
      private: ros::Publisher pubAtlasCommand;

      /// \brief publisher of AtlasSimInterfaceCommand
      private: ros::Publisher pubAtlasSimInterfaceCommand;

      /// \brief ros node handle
      private: ros::NodeHandle* rosNode;

      /// \brief local copy of AtlasCommand message
      private: atlas_msgs::AtlasCommand ac;

      /// \brief latest received JointStates from robot.
      private: sensor_msgs::JointState::ConstPtr js;
      private: bool js_valid;

      /// \brief hardcoded joint names for atlas
      private: std::vector<std::string> jointNames;

      /// \brief Atlas version number.
      private: int atlasVersion;

      /// \brief Atlas sub version number. This was added to handle two
      /// different versions of Atlas v4.
      /// atlasVersion == 4 && atlasSubVersion == 0: wry2 joints exist.
      /// atlasVersion == 4 && atlasSubVersion == 1: wry2 joints don't exist.
      private: int atlasSubVersion;

      friend class VRCPlugin;
    } atlasCommandController;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: bool warpRobotWithCmdVel;
    private: common::Time warpRobotStopTime;
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

    /// \brief Are cheats enabled?
    private: bool cheatsEnabled;

    /// \brief time out when receiving fake teleop cmd_vel command
    private: double cmdVelTopicTimeout;
  };
/** \} */
/// @}
}
#endif
