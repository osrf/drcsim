/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2012 Open Source Robotics Foundation
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
 * Desc: Plugin for controlling Sandia Hand model in gazebo
 * Author: John Hsu
 */
#ifndef __SANDIA_HAND_PLUGIN_HH_
#define __SANDIA_HAND_PLUGIN_HH_

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
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/Sensor.hh"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

  class SandiaHandPlugin : public ModelPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model
    public: SandiaHandPlugin();

    /// \brief Destructor
    public: ~SandiaHandPlugin();

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
    private: ros::Subscriber set_left_camera_frame_rate_sub_;
    private: void SetLeftCameraFrameRate(const std_msgs::Float64::ConstPtr
                                         &_msg);

    private: ros::Subscriber set_right_camera_frame_rate_sub_;
    private: void SetRightCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg);

    // ros services

    // gazebo variables
    private: physics::WorldPtr world;
    private: physics::ModelPtr atlasModel;
    private: sdf::ElementPtr sdf;
    private: common::Time lastTime;

    // camera control
    private: sensors::CameraSensorPtr leftCameraSensor;
    private: sensors::CameraSensorPtr rightCameraSensor;
    private: double leftCameraFrameRate;
    private: double rightCameraFrameRate;

    // laser sensor control
    private: sensors::RaySensorPtr laserSensor;

    /// Throttle update rate
    private: double lastUpdateTime;
    private: double updateRate;

  };

}
#endif

