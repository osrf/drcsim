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

#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/sdf/interface/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

namespace gazebo
{
  class GazeboRosLaser : public RayPlugin
  {
    /// \brief Constructor
    public: GazeboRosLaser();

    /// \brief Destructor
    public: ~GazeboRosLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewLaserScans();

    /// \brief Put laser data to the ROS topic
    private: void PutLaserData(common::Time &_updateTime);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief ros message
    private: sensor_msgs::LaserScan laser_msg_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief hack to mimic hokuyo intensity cutoff of 100
    private: double hokuyo_min_intensity_;

    /// update rate of this sensor
    private: double update_rate_;
    private: double update_period_;
    private: common::Time last_update_time_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue laser_queue_;
    private: void LaserQueueThread();
    private: boost::thread callback_queue_thread_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;
  };
}
#endif
