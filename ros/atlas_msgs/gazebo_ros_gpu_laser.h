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

#ifndef GAZEBO_ROS_GPU_LASER_HH
#define GAZEBO_ROS_GPU_LASER_HH

// system
#include <string>

// boost stuff
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// gazebo stuff
#include <gazebo/sdf/interface/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>

namespace gazebo
{
  class GazeboRosGpuLaser : public GpuRayPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosGpuLaser();

    /// \brief Destructor
    public: ~GazeboRosGpuLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: void Init();

    /// \brief Update the controller
    protected: virtual void OnNewLaserFrame(const float *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    //  protected: virtual void OnNewImageFrame(const unsigned char *_image,
    //                 unsigned int _width, unsigned int _height,
    //                 unsigned int _depth, unsigned int cam);
    //  protected: void PutCameraData(const unsigned char *_src, unsigned int w,
    //     unsigned int h, unsigned int d, image_transport::Publisher *pub_);
    //
    //  ///// \brief ROS image message
    //  protected: image_transport::Publisher image_pub_;
    //  protected: image_transport::Publisher image2_pub_;
    //  protected: image_transport::Publisher image3_pub_;
    //  protected: image_transport::Publisher image4_pub_;
    //  private: image_transport::ImageTransport* itnode_;
    //  /// \brief Keep track of number of connctions
    //  protected: int imageConnectCount;
    //  private: void ImageConnect();
    //  private: void ImageDisconnect();

    protected: void PublishLaserScan(const float *_scan,
      unsigned int _width);

    protected: void PublishPointCloud(const float *_scan,
      unsigned int _width, unsigned int _height);

    /// \brief Sets sensor update rate (not the plugin update rate).
    private: void GazeboRosGpuLaser::SetUpdateRate(double _rate);

    /// \brief Gaussian noise
    private: double gaussianNoise;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief hack to mimic hokuyo intensity cutoff of 100
    private: double hokuyoMinIntensity;

    /// \brief Keep track of number of connctions for point clouds
    private: int laserConnectCount;
    private: void LaserConnect();
    private: void LaserDisconnect();

    /// \brief A pointer to the ROS node.  A node will be instantiated
    /// if it does not exist.
    private: ros::Publisher pubLaserScan;

    /// \brief PCL point cloud message
    private: pcl::PointCloud<pcl::PointXYZI> pointCloudMsg;

    private: sensor_msgs::LaserScan laserScanMsg;

    private: double pointCloudCutoff;

    /// \brief ROS image topic name
    private: std::string laserTopicName;

    // overload with our own
    private: common::Time sensorUpdateTime;

    protected: ros::NodeHandle* rosnode;
    private: std::string robotNamespace;

    protected: ros::CallbackQueue queue;
    protected: void QueueThread();
    protected: boost::thread callbackQueueThread;

    protected: ros::WallTime lastPubTime;

    protected: std::string frameName;
    protected: double updateRate;
    protected: double updatePeriod;
  };
}
#endif

