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

#ifndef GAZEBO_ATLAS_PLUGIN_HH
#define GAZEBO_ATLAS_PLUGIN_HH

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
#include <atlas_msgs/SynchronizationStatistics.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

// AtlasSimInterface: header
#include "AtlasSimInterface.h"

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

// publish separate /atlas/imu topic, to be deprecated
#include "sensor_msgs/Imu.h"
// publish separate /atlas/force_torque_sensors topic, to be deprecated
#include <atlas_msgs/ForceTorqueSensors.h>

#include <atlas_msgs/ResetControls.h>
#include <atlas_msgs/SetJointDamping.h>
#include <atlas_msgs/GetJointDamping.h>
#include <atlas_msgs/ControllerStatistics.h>

// don't use these to control
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

// high speed control
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>

// low speed control
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>

#include <atlas_msgs/Test.h>

#include "PubQueue.h"

namespace gazebo
{
  class AtlasPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: AtlasPlugin();

    /// \brief Destructor
    public: virtual ~AtlasPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief connected by lContactUpdateConnection, called when contact
    /// sensor update
    private: void OnLContactUpdate();

    /// \brief connected by rContactUpdateConnection, called when contact
    /// sensor update
    private: void OnRContactUpdate();

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief get data from IMU for robot state
    /// \param[in] _curTime current simulation time
    private: void GetIMUState(const common::Time &_curTime);

    /// \brief get data from force torque sensor
    private: void GetForceTorqueSensorState(const common::Time &_curTime);

    /// \brief ros service callback to reset joint control internal states
    /// \param[in] _req Incoming ros service request
    /// \param[in] _res Outgoing ros service response
    private: bool ResetControls(atlas_msgs::ResetControls::Request &_req,
      atlas_msgs::ResetControls::Response &_res);

    /// \brief ros service callback to set joint damping
    /// \param[in] _req Incoming ros service request
    /// \param[in] _res Outgoing ros service response
    private: bool SetJointDamping(atlas_msgs::SetJointDamping::Request &_req,
      atlas_msgs::SetJointDamping::Response &_res);

    /// \brief ros service callback to get joint damping
    /// \param[in] _req Incoming ros service request
    /// \param[in] _res Outgoing ros service response
    private: bool GetJointDamping(atlas_msgs::GetJointDamping::Request &_req,
      atlas_msgs::GetJointDamping::Response &_res);

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief pointer to gazebo world
    private: physics::WorldPtr world;

    /// \brief pointer to gazebo Atlas model
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;
    private: event::ConnectionPtr rContactUpdateConnection;
    private: event::ConnectionPtr lContactUpdateConnection;

    /// Throttle update rate
    private: common::Time lastControllerStatisticsTime;
    private: double statsUpdateRate;

    // Contact sensors
    private: sensors::ContactSensorPtr lFootContactSensor;
    private: sensors::ContactSensorPtr rFootContactSensor;
    private: ros::Publisher pubLFootContact;
    private: PubQueue<geometry_msgs::WrenchStamped>::Ptr pubLFootContactQueue;
    private: ros::Publisher pubRFootContact;
    private: PubQueue<geometry_msgs::WrenchStamped>::Ptr pubRFootContactQueue;

    // Force torque sensors at ankles
    private: physics::JointPtr rAnkleJoint;
    private: physics::JointPtr lAnkleJoint;

    // Force torque sensors at the wrists
    private: physics::JointPtr rWristJoint;
    private: physics::JointPtr lWristJoint;

    /// \brief A combined AtlasState, IMU and ForceTorqueSensors Message
    /// for accessing all these states synchronously.
    private: atlas_msgs::AtlasState atlasState;
    private: void GetAndPublishRobotStates(const common::Time &_curTime);

    // IMU sensor
    private: boost::shared_ptr<sensors::ImuSensor> imuSensor;
    private: std::string imuLinkName;
    // publish separate /atlas/imu topic, to be deprecated
    private: ros::Publisher pubImu;
    private: PubQueue<sensor_msgs::Imu>::Ptr pubImuQueue;

    /// \brief ros publisher for force torque sensors
    private: ros::Publisher pubForceTorqueSensors;
    private: PubQueue<atlas_msgs::ForceTorqueSensors>::Ptr
      pubForceTorqueSensorsQueue;

    /// Deferred loading in case ros is blocking, \TODO: not working for now.
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS internal stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;

    /// \brief ros publisher for ros controller timing statistics
    private: ros::Publisher pubControllerStatistics;
    private: PubQueue<atlas_msgs::ControllerStatistics>::Ptr
      pubControllerStatisticsQueue;

    /// \brief ROS publisher for atlas joint states
    private: ros::Publisher pubJointStates;
    private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

    /// \brief ROS publisher for atlas state, currently it contains
    /// joint index enums
    /// atlas_msgs::AtlasState
    private: ros::Publisher pubAtlasState;
    private: PubQueue<atlas_msgs::AtlasState>::Ptr pubAtlasStateQueue;

    private: ros::Subscriber subAtlasCommand;
    private: ros::Subscriber subJointCommands;

    /// \brief ros topic callback to update Atlas Commands
    /// \param[in] _msg Incoming ros message
    private: void SetAtlasCommand(
      const atlas_msgs::AtlasCommand::ConstPtr &_msg);

    /// \brief ros topic callback to update Joint Commands slowly.
    /// Control conmmands received through /atlas/joint_commands are
    /// not expected to be able to close near 1kHz.
    /// \param[in] _msg Incoming ros message
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg);

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //  Controller Synchronization Control                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief Step simulation once.
    private: void Tic(const std_msgs::String::ConstPtr &_msg);

    /// \brief Special topic for advancing simulation by a ROS topic.
    private: ros::Subscriber subTic;

    /// \brief Condition variable for tic-ing simulation step.
    private: boost::condition delayCondition;

    /// \brief a non-moving window is used, every delayWindowSize-seconds
    /// the user is allotted delayMaxPerWindow seconds of delay budget.
    private: common::Time delayWindowSize;

    /// \brief Marks the start of a non-moving delay window.
    private: common::Time delayWindowStart;

    /// \brief Within each window, simulation will wait at
    /// most a total of delayMaxPerWindow seconds.
    private: common::Time delayMaxPerWindow;

    /// \brief Within each simulation step, simulation will wait at
    /// most delayMaxPerStep seconds to receive information from controller.
    private: common::Time delayMaxPerStep;

    /// \brief Within each window, simulation will wait at
    /// most a total of delayMaxPerWindow seconds.
    private: common::Time delayInWindow;

    /// \brief Publish controller synchronization delay information.
    private: ros::Publisher pubDelayStatistics;
    private: PubQueue<atlas_msgs::SynchronizationStatistics>::Ptr
      pubDelayStatisticsQueue;
    private: atlas_msgs::SynchronizationStatistics delayStatistics;

    /// \brief enforce delay policy
    private: void EnforceSynchronizationDelay(const common::Time &_curTime);

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //  BDI Controller AtlasSimInterface Internals                            //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    // AtlasSimInterface:
    private: AtlasControlOutput controlOutput;
    private: AtlasRobotState atlasRobotState;
    private: AtlasControlInput atlasControlInput;
    private: AtlasSimInterface* atlasSimInterface;

    /// \brief AtlasSimInterface: ROS subscriber
    private: ros::Subscriber subASICommand;
    /// \brief AtlasSimInterface: ROS callback
    private: void SetASICommand(
      const atlas_msgs::AtlasSimInterfaceCommand::ConstPtr &_msg);
    private: ros::Publisher pubASIState;
    private: PubQueue<atlas_msgs::AtlasSimInterfaceState>::Ptr pubASIStateQueue;
    private: boost::mutex asiMutex;

    /// \brief internal copy of atlasSimInterfaceState
    private: atlas_msgs::AtlasSimInterfaceState asiState;

    /// \brief helper functions converting behavior string to int
    private: std::map<std::string, int> behaviorMap;

    /// \brief helper functions converting behavior int to string
    private: std::string GetBehavior(int _behavior);

    /// \brief helper function to copy states
    private: void AtlasControlOutputToAtlasSimInterfaceState(
              atlas_msgs::AtlasSimInterfaceState *_fb,
              AtlasControlOutput *_fbOut);

    // AtlasSimInterface:  Controls ros interface
    private: ros::Subscriber subAtlasControlMode;

    /// \brief AtlasSimInterface:
    /// subscribe to a control_mode string message, current valid commands are:
    ///   walk, stand, safety, stand-prep, none
    /// the command is passed to the AtlasSimInterface library.
    /// \param[in] _mode Can be "walk", "stand", "safety", "stand-prep", "none".
    private: void OnRobotMode(const std_msgs::String::ConstPtr &_mode);

    /// \brief Process BDI contoller updates
    /// \param[in] _curTime current simulation time
    private: void UpdateAtlasSimInterface(const common::Time &_curTime);

    /// \brief Update PID Joint Servo Controllers
    /// \param[in] _dt time step size since last update
    private: void UpdatePIDControl(double _dt);

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //  Some Helper Functions                                                 //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: void LoadPIDGainsFromParameter();
    private: void ZeroAtlasCommand();
    private: void ZeroJointCommands();

    /// \brief keep a list of hard coded Atlas joint names.
    private: std::vector<std::string> jointNames;

    /// \brief internal bufferred controller states
    private: atlas_msgs::AtlasCommand atlasCommand;
    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState jointStates;

    // JointController: pointer to a copy of the joint controller in gazebo
    // \TODO: not yet functional
    private: physics::JointControllerPtr jointController;
    private: transport::NodePtr node;
    private: transport::PublisherPtr jointCmdPub;

    /// \brief Internal list of pointers to Joints
    private: physics::Joint_V joints;
    private: std::vector<double> effortLimit;

    /// \brief internal class for keeping track of PID states
    private: class ErrorTerms
      {
        /// error term contributions to final control output
        double q_p;
        double d_q_p_dt;
        double k_i_q_i;  // integral term weighted by k_i
        double qd_p;
        friend class AtlasPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: boost::mutex mutex;

    /// \brief ros service to reset controls internal states
    private: ros::ServiceServer resetControlsService;

    /// \brief: for keeping track of internal controller update rates.
    private: common::Time lastControllerUpdateTime;

    /// \brief ros service to change joint damping
    private: ros::ServiceServer setJointDampingService;

    /// \brief ros service to retrieve joint damping
    private: ros::ServiceServer getJointDampingService;


    /// \brief Conversion functions
    private: inline math::Pose ToPose(const geometry_msgs::Pose &_pose) const
    {
      return math::Pose(math::Vector3(_pose.position.x,
                                      _pose.position.y,
                                      _pose.position.z),
                        math::Quaternion(_pose.orientation.w,
                                         _pose.orientation.x,
                                         _pose.orientation.y,
                                         _pose.orientation.z));
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Pose ToPose(const math::Pose &_pose) const
    {
      geometry_msgs::Pose result;
      result.position.x = _pose.pos.x;
      result.position.y = _pose.pos.y;
      result.position.z = _pose.pos.y;
      result.orientation.w = _pose.rot.w;
      result.orientation.x = _pose.rot.x;
      result.orientation.y = _pose.rot.y;
      result.orientation.z = _pose.rot.z;
      return result;
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Point ToPoint(const AtlasVec3f &_v) const
    {
      geometry_msgs::Point result;
      result.x = _v.n[0];
      result.y = _v.n[1];
      result.z = _v.n[2];
      return result;
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Quaternion ToQ(const math::Quaternion &_q)
      const
    {
      geometry_msgs::Quaternion result;
      result.w = _q.w;
      result.x = _q.x;
      result.y = _q.y;
      result.z = _q.z;
      return result;
    }

    /// \brief Conversion helper functions
    private: inline AtlasVec3f ToVec3(const geometry_msgs::Point &_point) const
    {
      return AtlasVec3f(_point.x,
                        _point.y,
                        _point.z);
    }

    /// \brief Conversion helper functions
    private: inline AtlasVec3f ToVec3(const math::Vector3 &_vector3) const
    {
      return AtlasVec3f(_vector3.x,
                        _vector3.y,
                        _vector3.z);
    }

    /// \brief Conversion helper functions
    private: inline math::Vector3 ToVec3(const AtlasVec3f &_vec3) const
    {
      return math::Vector3(_vec3.n[0],
                           _vec3.n[1],
                           _vec3.n[2]);
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Vector3 ToGeomVec3(
      const AtlasVec3f &_vec3) const
    {
      geometry_msgs::Vector3 result;
      result.x = _vec3.n[0];
      result.y = _vec3.n[1];
      result.z = _vec3.n[2];
      return result;
    }

    // controls message age measure
    private: atlas_msgs::ControllerStatistics controllerStatistics;
    private: std::vector<double> atlasCommandAgeBuffer;
    private: std::vector<double> atlasCommandAgeDelta2Buffer;
    private: unsigned int atlasCommandAgeBufferIndex;
    private: double atlasCommandAgeBufferDuration;
    private: double atlasCommandAgeMean;
    private: double atlasCommandAgeVariance;
    private: double atlasCommandAge;
    private: void CalculateControllerStatistics(const common::Time &_curTime);
    private: void PublishConstrollerStatistics(const common::Time &_curTime);

    private: void SetExperimentalDampingPID(
      const atlas_msgs::Test::ConstPtr &_msg);
    private: ros::Subscriber subTest;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;

    /// \brief Are cheats enabled?
    private: bool cheatsEnabled;

    /// \brief joint damping coefficient bounds
    private: std::vector<double> jointDampingMax;
    private: std::vector<double> jointDampingMin;

    /// \brief kp_velocity as cfm damping upper bound
    private: double kp_velocityMax;

    /// \brief kp_velocity as cfm damping lower bound
    private: double kp_velocityMin;
  };
}
#endif
