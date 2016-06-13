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

#ifndef GAZEBO_SANDIA_HAND_PLUGIN_HH
#define GAZEBO_SANDIA_HAND_PLUGIN_HH

#include <string>
#include <vector>
#include <list>

#include <boost/thread/mutex.hpp>
#include <boost/unordered/unordered_map.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <atlas_msgs/SetJointDamping.h>
#include <atlas_msgs/GetJointDamping.h>
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
#include <sensor_msgs/JointState.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  namespace physics {
    class Collision;
  }

  class SandiaHandPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SandiaHandPlugin();

    /// \brief Destructor
    public: virtual ~SandiaHandPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();
    private: void CopyVectorIfValid(const std::vector<double> &from,
                                    std::vector<double> &to);

    /// \brief Callback for contact messages
    /// \param[in] _msg Gazebo contact message
    private: void OnContacts(ConstContactsPtr &_msg);

    typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

    /// \brief Fill ROS tactile message using Gazebo contact message
    /// \param [in] _incomingContacts Gazebo contact message
    /// \param[in] _tactileMsg ROS tactile message
    private: void FillTactileData(ContactMsgs_L _incomingContacts,
        sandia_hand_msgs::RawTactile *_tactileMsg);

    /// \brief ROS callback when a subscriber connects to tactile
    /// publisher
    private: void TactileConnect();

    /// \brief ROS callback when a subscriber disconnects from tactile
    /// publisher
    private: void TactileDisconnect();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Which hand (left/right)
    private: std::string side;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: double lastStatusTime;
    private: double updateRate;

    // IMU sensor
    private: sensors::ImuSensorPtr ImuSensor;
    private: common::Time lastImuTime;
    private: std::string ImuLinkName;
    private: physics::LinkPtr ImuLink;
    private: ros::Publisher pubImu;
    private: PubQueue<sensor_msgs::Imu>::Ptr pubImuQueue;

    // tactile sensor
    /// \brief ROS publisher for the tactile message
    private: ros::Publisher pubTactile;

    /// \brief ROS tactile message publisher queue
    private: PubQueue<sandia_hand_msgs::RawTactile>::Ptr pubTactileQueue;

    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;
    private: ros::Publisher pubJointStates;
    private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

    private: ros::Subscriber subJointCommands;
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg);

    private: std::vector<std::string> jointNames;
    private: physics::Joint_V joints;
    private: class ErrorTerms
      {
        double q_p;
        double d_q_p_dt;
        double q_i;
        double qd_p;
        friend class SandiaHandPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState jointStates;

    /// \brief ROS tactile message to be published
    private: sandia_hand_msgs::RawTactile tactile;

    // Controls stuff
    private: common::Time lastControllerUpdateTime;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue* pmq;

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

    /// \brief ros service to change joint damping
    private: ros::ServiceServer setJointDampingService;

    /// \brief ros service to retrieve joint damping
    private: ros::ServiceServer getJointDampingService;

    /// \brief joint damping coefficient bounds
    private: std::vector<double> jointDampingMax;
    private: std::vector<double> jointDampingMin;

    /// \breif prevent overwriting commadns
    private: boost::mutex mutex;

    // flag to indicate that stumps are in use
    private: bool hasStumps;

    /// \brief Subscription to contact messages
    private: transport::SubscriberPtr contactSub;

    /// \brief Incoming Gazebo contact messages
    private: ContactMsgs_L incomingContacts;

    /// \brief Transport node used for subscribing to contact sensor messages.
    private: transport::NodePtr node;

    /// \brief Mutex to protect reads and writes.
    private: mutable boost::mutex contactMutex;

    /// \brief List of contact collisions used for generating tactile sensor
    /// output
    private: boost::unordered_map<std::string, physics::Collision *>
                contactCollisions;

    /// \brief Length of finger collision:
    /// index 0 is the lower link collision,
    /// index 1 is the upper link collision.
    private: double fingerColLength[2];

    /// \brief Width of finger collision:
    /// index 0 is the lower link collision,
    /// index 1 is the upper link collision.
    private: double fingerColWidth[2];

    /// \brief Tactile sensor column size on the finger collision
    private: int fingerHorSize[2];

    /// \brief Tactile sensor row size on the finger collision
    private: int fingerVerSize[2];

    /// \brief Width of palm collision:
    /// index 0 is the index finger palm collision,
    /// index 1 is the middle finger palm collision,
    /// index 2 is the pinky palm collision,
    /// index 3 is the bottom palm collision,
    /// index 4 is the mid palm collision.
    private: double palmColWidth[5];

    /// \brief Length of palm collision:
    /// index 0 is the index finger palm collision,
    /// index 1 is the middle finger palm collision,
    /// index 2 is the pinky palm collision,
    /// index 3 is the bottom palm collision,
    /// index 4 is the mid palm collision.
    private: double palmColLength[5];

    /// \brief Tactile sensor column size on the palm collision.
    private: int palmHorSize[5];

    /// \brief Tactile sensor row size on the palm collision.
    private: int palmVerSize[5];

    /// \brief Total number of tactile sensors on each finger.
    private: int tactileFingerArraySize;

    /// \brief Total number of tactile sensors on the palm.
    private: int tactilePalmArraySize;

    /// \brief Max tactile sensor data output
    private: int maxTactileOut;

    /// \brief min tactile sensor data output
    private: int minTactileOut;

    /// \brief Keep track of number of tactile sensor connections
    private: int tactileConnectCount;

    /// \brief Mutex to protect tactileConnectcount
    private: boost::mutex tactileConnectionMutex;
  };
/** \} */
/// @}
}
#endif
