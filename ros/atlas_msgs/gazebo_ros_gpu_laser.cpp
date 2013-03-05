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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <assert.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sdf/interface/SDF.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <tf/tf.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

#include "gazebo_ros_gpu_laser.h"


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpuLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGpuLaser::GazeboRosGpuLaser()
{
  this->laserConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGpuLaser::~GazeboRosGpuLaser()
{
  this->parentSensor->SetActive(false);
  this->rosnode->shutdown();
  this->queue.clear();
  this->queue.disable();
  this->callbackQueueThread.join();
  delete this->rosnode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpuLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GpuRayPlugin::Load(_parent, _sdf);

  //GazeboRosCameraUtils::Load(_parent, _sdf);
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  // point cloud stuff
  if (!_sdf->GetElement("topicName"))
    this->laserTopicName = "scan";
  else
    this->laserTopicName = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->GetElement("pointCloudCutoff"))
    this->pointCloudCutoff = 0.4;
  else
    this->pointCloudCutoff = _sdf->GetElement("pointCloudCutoff")->GetValueDouble();

  this->frameName = "/world";
  if (_sdf->HasElement("frameName"))
    this->frameName = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Block laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussianNoise = 0;
  }
  else
    this->gaussianNoise = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("hokuyoMinIntensity"))
  {
    ROS_INFO("Block laser plugin missing <hokuyoMinIntensity>, defaults to 101");
    this->hokuyoMinIntensity = 101;
  }
  else
    this->hokuyoMinIntensity = _sdf->GetElement("hokuyoMinIntensity")->GetValueDouble();

  ROS_INFO("INFO: gazebo_ros_laser plugin should set minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyoMinIntensity);

  if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Camera plugin missing <updateRate>, defaults to 0");
    this->updateRate = 0;
  }
  else
    this->updateRate = _sdf->GetElement("updateRate")->GetValueDouble();


  // Exit if no ROS
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  // resolve tf prefix
  std::string prefix;
  this->rosnode->getParam(std::string("tf_prefix"), prefix);
  this->frameName = tf::resolve(prefix, this->frameName);

  ros::AdvertiseOptions laser_scan_ao;

  if (this->parentSensor->GetVerticalRangeCount() != 1)
    laser_scan_ao = ros::AdvertiseOptions::create<pcl::PointCloud<pcl::PointXYZI> >(
      this->laserTopicName,1,
      boost::bind( &GazeboRosGpuLaser::LaserConnect,this),
      boost::bind( &GazeboRosGpuLaser::LaserDisconnect,this),
      ros::VoidPtr(), &this->queue);
  else
    laser_scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->laserTopicName,1,
      boost::bind( &GazeboRosGpuLaser::LaserConnect,this),
      boost::bind( &GazeboRosGpuLaser::LaserDisconnect,this),
      ros::VoidPtr(), &this->queue);
  this->pubLaserScan = this->rosnode->advertise(laser_scan_ao);

  this->Init();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::SetUpdateRate(double _rate)
{
  // set parent sensor update rate
  this->parentSensor->SetUpdateRate(_rate);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::Init()
{
  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->updateRate > 0.0)
    this->updatePeriod = 1.0/this->updateRate;
  else
    this->updatePeriod = 0.0;

  // start custom queue for camera_
  this->callbackQueueThread = boost::thread(boost::bind(&GazeboRosGpuLaser::QueueThread, this));

  this->lastPubTime = ros::WallTime::now();
  // this->logCount_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosGpuLaser::LaserConnect()
{
  this->laserConnectCount++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosGpuLaser::LaserDisconnect()
{
  this->laserConnectCount--;

  if (this->laserConnectCount <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGpuLaser::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  this->sensorUpdateTime = this->parentSensor->GetLastUpdateTime();
  if (!this->parentSensor->IsActive())
  {
    if (this->laserConnectCount > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->laserConnectCount > 0)
    {
      if (this->parentSensor->GetVerticalRangeCount() == 1)
        PublishLaserScan(_image, width);
      else
        PublishPointCloud(_image, width, height);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::PublishLaserScan(const float *_scan,
    unsigned int _width)
{
  math::Angle maxAngle = this->parentSensor->GetAngleMax();
  math::Angle minAngle = this->parentSensor->GetAngleMin();

  this->laserScanMsg.header.frame_id = this->frameName;
  this->laserScanMsg.header.stamp.sec = this->sensorUpdateTime.sec;
  this->laserScanMsg.header.stamp.nsec = this->sensorUpdateTime.nsec;

  this->laserScanMsg.angle_min = minAngle.GetAsRadian();
  this->laserScanMsg.angle_max = maxAngle.GetAsRadian();
  this->laserScanMsg.angle_increment = (maxAngle.GetAsRadian() - minAngle.GetAsRadian())/((double)(_width -1)); // for computing yaw
  this->laserScanMsg.time_increment  = 0; // instantaneous simulator scan
  this->laserScanMsg.scan_time       = 0; // FIXME: what's this?
  this->laserScanMsg.range_min = this->parentSensor->GetRangeMin();
  this->laserScanMsg.range_max = this->parentSensor->GetRangeMax();
  this->laserScanMsg.ranges.clear();
  this->laserScanMsg.intensities.clear();

  for(unsigned int i=0; i < width; i++)
  {
    float range = _scan[3 * i];
    if (range < this->parentSensor->GetRangeMin())
      range = this->parentSensor->GetRangeMax();
    this->laserScanMsg.ranges.push_back(range + this->GaussianKernel(0,this->gaussianNoise));
    this->laserScanMsg.intensities.push_back(_scan[3*i+1]);
  }

  this->pubLaserScan.publish(this->laserScanMsg);

}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::PublishPointCloud(const float *_scan,
    unsigned int _width, unsigned int _height)
{
  this->pointCloudMsg.header.frame_id = this->frameName;
  this->pointCloudMsg.header.stamp.sec = this->sensorUpdateTime.sec;
  this->pointCloudMsg.header.stamp.nsec = this->sensorUpdateTime.nsec;

  this->pointCloudMsg.points.clear();
  this->pointCloudMsg.is_dense = true;

  math::Angle maxAngle = this->parentSensor->GetAngleMax();
  math::Angle minAngle = this->parentSensor->GetAngleMin();

  math::Angle verticalMaxAngle = this->parentSensor->GetVerticalAngleMax();
  math::Angle verticalMinAngle = this->parentSensor->GetVerticalAngleMin();

  double dH = (maxAngle.GetAsRadian() - minAngle.GetAsRadian()) / (_width - 1) ;
  double dV = (verticalMaxAngle.GetAsRadian() - verticalMinAngle.GetAsRadian()) / (_height - 1);

  double alpha = ((minAngle + maxAngle) / 2.0).GetAsRadian();

  for (unsigned int j = 0; j < _height; j++)
    for (unsigned int i = 0; i < _width; i++)
    {
      double hAngle = (i * dH) + minAngle.GetAsRadian() - alpha;
      double vAngle = (j * dV) + verticalMinAngle.GetAsRadian();
      float r = _scan[3 * (i + j * _width)];

      pcl::PointXYZI pr;

      if ((r < this->parentSensor->GetRangeMin()) ||
          (r >= this->parentSensor->GetRangeMax()))
        r = this->parentSensor->GetRangeMax();

      pcl::PointXYZI p;

      if (this->parentSensor->IsHorizontal())
      {
        p.x = r * cos(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussianNoise);
        p.y = r *               sin(hAngle) + this->GaussianKernel(0,this->gaussianNoise);
        p.z = r * sin(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussianNoise);
      }
      else
      {
        p.x = r * cos(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussianNoise);
        p.y = r * cos(vAngle) * sin(hAngle) + this->GaussianKernel(0,this->gaussianNoise);
        p.z = r * sin(vAngle) + this->GaussianKernel(0,this->gaussianNoise);
      }

      pr.x = cos(alpha)*p.x - sin(alpha)*p.y;
      pr.y = sin(alpha)*p.x + cos(alpha)*p.y;
      pr.z = p.z;
      pr.intensity = _scan[3 * (i + j * _width) + 1];

      this->pointCloudMsg.points.push_back(pr);
    }

  this->pubLaserScan.publish(this->pointCloudMsg);
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosGpuLaser::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosGpuLaser::QueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode->ok())
  {
    /// take care of callback queue
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}
}
