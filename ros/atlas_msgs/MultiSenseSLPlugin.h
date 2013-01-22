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

#ifndef __MULTISENSE_SL_PLUGIN_HH_
#define __MULTISENSE_SL_PLUGIN_HH_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "std_srvs/Empty.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "transport/TransportTypes.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/Sensor.hh"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

  class MultiSenseSL : public ModelPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model
    public: MultiSenseSL();

    /// \brief Destructor
    public: ~MultiSenseSL();

    /// \brief Load the plugin
    /// \param[in] _parent pointer to parent Model
    /// \param[in] _sdf SDF root element corresponds to the plugin XML block
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller periodically via Events.
    protected: virtual void UpdateStates();

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Thread for loading and initializing ROS
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // reset of ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // ros topics publisher
    private: ros::Publisher pub_status_;

    // ros topic subscriber
    private: ros::Subscriber set_spindle_speed_sub_;
    private: void SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg);

    private: ros::Subscriber set_spindle_state_sub_;
    private: void SetSpindleState(const std_msgs::Bool::ConstPtr &_msg);

    private: ros::Subscriber set_multi_camera_frame_rate_sub_;
    private: void SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                         &_msg);

    private: ros::Subscriber set_multi_camera_exposure_time_sub_;
    private: void SetMultiCameraExposureTime(const std_msgs::Float64::ConstPtr
                                            &_msg);

    private: ros::Subscriber set_multi_camera_gain_sub_;
    private: void SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                    &_msg);

    // ros services
    private: ros::ServiceServer set_spindle_state_service_;
    private: bool SetSpindleState(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    private: ros::ServiceServer set_spindle_speed_service_;
    private: bool SetSpindleSpeed(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    // gazebo variables
    private: physics::WorldPtr world;
    private: physics::ModelPtr atlasModel;
    private: sdf::ElementPtr sdf;
    private: common::Time lastTime;

    // camera control
    private: boost::shared_ptr<sensors::MultiCameraSensor> multiCameraSensor;
    private: double multiCameraFrameRate;
    private: double multiCameraExposureTime;
    private: double multiCameraGain;

    // laser sensor control
    private: sensors::RaySensorPtr laserSensor;

    // spindle control
    private: double spindleSpeed;
    private: double spindleMaxRPM;
    private: double spindleMinRPM;
    private: bool spindleOn;
    private: physics::LinkPtr spindleLink;
    private: physics::JointPtr spindleJoint;
    private: common::PID spindlePID;

    /// Throttle update rate
    private: double lastUpdateTime;
    private: double updateRate;

  };

}
#endif

