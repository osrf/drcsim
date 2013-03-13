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
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>

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

#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/ResetControls.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <atlas_msgs/ControllerStatistics.h>
#include <atlas_msgs/AtlasState.h>
#include <sensor_msgs/JointState.h>

#include <atlas_msgs/Test.h>

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

    /// \brief ros service callback to reset joint control internal states
    /// \param[in] _req Incoming ros service request
    /// \param[in] _res Outgoing ros service response
    private: bool ResetControls(atlas_msgs::ResetControls::Request &_req,
      atlas_msgs::ResetControls::Response &_res);

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    private: physics::WorldPtr world;
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
    private: ros::Publisher pubRFootContact;

    // Force torque sensors at ankles
    private: physics::JointPtr rAnkleJoint;
    private: physics::JointPtr lAnkleJoint;

    // Force torque sensors at the wrists
    private: physics::JointPtr rWristJoint;
    private: physics::JointPtr lWristJoint;

    /// \brief A combined JointStates, IMU and ForceTorqueSensors Message
    /// for accessing all these states synchronously.
    private: atlas_msgs::AtlasState atlasState;

    // IMU sensor
    private: boost::shared_ptr<sensors::ImuSensor> imuSensor;
    private: std::string imuLinkName;
    private: physics::LinkPtr imuLink;
    private: ros::Publisher pubImu;
    private: common::Time lastImuTime;

    // AtlasSimInterface: internal debugging only
    // Pelvis position and velocity
    private: std::string pelvisLinkName;
    private: physics::LinkPtr pelvisLink;

    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS internal stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;

    /// \brief ros publisher for ros controller timing statistics
    private: ros::Publisher pubControllerStatistics;

    /// \brief ros publisher for force atlas joint states
    private: ros::Publisher pubJointStates;

    /// \brief ros publisher for force torque sensors
    private: ros::Publisher pubForceTorqueSensors;

    /// \brief ros publisher for atlas states, currently it contains
    /// joint index enums
    /// sensor_msgs::JointState
    /// sensor_msgs::Imu
    /// atlas_msgs::FroceTorqueSensors
    private: ros::Publisher pubAtlasState;

    private: ros::Subscriber subJointCommands;

    /// \brief ros topic callback to update Joint Commands
    /// \param[in] _msg Incoming ros message
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg);

    /// \brief ros topic callback to update Joint Commands
    /// \param[in] _msg Incoming ros message
    private: void UpdateJointCommands(
      const osrf_msgs::JointCommands &_msg);

    private: void LoadPIDGainsFromParameter();
    private: void ZeroJointCommands();

    private: std::vector<std::string> jointNames;

    // JointController: pointer to a copy of the joint controller in gazebo
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

    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState jointStates;
    private: boost::mutex mutex;

    /// \brief ros service to reset controls internal states
    private: ros::ServiceServer resetControlsService;

    // AtlasSimInterface:  Controls ros interface
    private: ros::Subscriber subAtlasControlMode;

    /// \brief AtlasSimInterface:
    /// subscribe to a control_mode string message, current valid commands are:
    ///   walk, stand, safety, stand-prep, none
    /// the command is passed to the AtlasSimInterface library.
    /// \param[in] _mode Can be "walk", "stand", "safety", "stand-prep", "none".
    private: void OnRobotMode(const std_msgs::String::ConstPtr &_mode);

    /// \brief internal variable for keeping state of the BDI walking controller
    private: bool usingWalkingController;

    /// \brief: for keeping track of internal controller update rates.
    private: common::Time lastControllerUpdateTime;

    // controls message age measure
    private: atlas_msgs::ControllerStatistics controllerStatistics;
    private: std::vector<double> jointCommandsAgeBuffer;
    private: std::vector<double> jointCommandsAgeDelta2Buffer;
    private: unsigned int jointCommandsAgeBufferIndex;
    private: double jointCommandsAgeBufferDuration;
    private: double jointCommandsAgeMean;
    private: double jointCommandsAgeVariance;
    private: double jointCommandsAge;

    private: void SetExperimentalDampingPID(
      const atlas_msgs::Test::ConstPtr &_msg);
    private: ros::Subscriber subTest;
  };
}
#endif
