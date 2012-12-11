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
 * Desc: A dynamic controller plugin that publishes ROS image_raw camera_info topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_DEPTH_CAMERA_HH
#define GAZEBO_ROS_DEPTH_CAMERA_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "sensor_msgs/fill_image.h"
#include <std_msgs/Float64.h>
#include "image_transport/image_transport.h"

// gazebo stuff
#include "sdf/interface/Param.hh"
#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "sensors/SensorTypes.hh"
#include "plugins/DepthCameraPlugin.hh"

// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

// boost stuff
#include "boost/thread/mutex.hpp"

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosDepthCamera : public DepthCameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosDepthCamera();

    /// \brief Destructor
    public: ~GazeboRosDepthCamera();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewDepthFrame(const float *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    protected: virtual void OnNewRGBPointCloud(const float *_pcd,
                    unsigned int _width, unsigned int _height,
                    unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    protected: virtual void OnNewImageFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Put camera data to the ROS topic
    private: void FillPointdCloud(const float *_src);

    /// \brief Keep track of number of connctions for point clouds
    private: int point_cloud_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();

    private: bool FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg,
                                  uint32_t rows_arg, uint32_t cols_arg,
                                  uint32_t step_arg, void* data_arg);

    private: bool FillDepthImageHelper( sensor_msgs::Image& image_msg,
                                  uint32_t rows_arg, uint32_t cols_arg,
                                  uint32_t step_arg, void* data_arg);

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    private: ros::Publisher point_cloud_pub_;
    private: ros::Publisher depth_image_pub_;

    /// \brief PCL point cloud message
    private: sensor_msgs::PointCloud2 point_cloud_msg_;
    private: sensor_msgs::Image depth_image_msg_;

    private: double point_cloud_cutoff_;

    /// \brief ROS image topic name
    private: std::string point_cloud_topic_name_;

    private: void InfoConnect();
    private: void InfoDisconnect();
    using GazeboRosCameraUtils::PublishCameraInfo;
    private: void PublishCameraInfo();

    /// \brief image where each pixel contains the depth information
    private: std::string depth_image_topic_name_;
    private: std::string depth_image_camera_info_topic_name_;

    // overload with our own
    private: common::Time depth_sensor_update_time_;
    protected: ros::Publisher depth_image_camera_info_pub_;
  };

}
#endif

