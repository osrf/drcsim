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

#include <string>
#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sdf/interface/SDF.hh>
#include <gazebo/sdf/interface/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Camera.hh>

#include "gazebo_ros_camera_utils.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCameraUtils::GazeboRosCameraUtils()
{
  this->image_connect_count_ = 0;
  this->last_update_time_ = common::Time(0);
  this->last_info_update_time_ = common::Time(0);
}

#ifdef DYNAMIC_RECONFIGURE
void GazeboRosCameraUtils::configCallback(
  gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f",
    this->camera_name_.c_str(), config.imager_rate);
  this->parentSensor_->SetUpdateRate(config.imager_rate);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->rosnode_->shutdown();
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  // Get the world name.
  std::string world_name = _parent->GetWorldName();

  // Get the world_
  this->world_ = physics::get_world(world_name);

  // save pointers
  this->sdf = _sdf;

  // maintain for one more release for backwards compatibility with
  // pr2_gazebo_plugins
  this->world = this->world_;

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->GetValueString("robotNamespace") + "/";

  this->image_topic_name_ = "image_raw";
  if (this->sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = this->sdf->GetValueString("imageTopicName");

  this->camera_info_topic_name_ = "camera_info";
  if (this->sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ =
      this->sdf->GetValueString("cameraInfoTopicName");

  if (!this->sdf->HasElement("cameraName"))
    ROS_INFO("Camera plugin missing <cameraName>, default to empty");
  else
    this->camera_name_ = this->sdf->GetValueString("cameraName");

  if (!this->sdf->HasElement("frameName"))
    ROS_INFO("Camera plugin missing <frameName>, defaults to /world");
  else
    this->frame_name_ = this->sdf->GetValueString("frameName");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO("Camera plugin missing <updateRate>, defaults to unlimited (0).");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->GetValueDouble("updateRate");

  if (!this->sdf->HasElement("CxPrime"))
  {
    ROS_INFO("Camera plugin missing <CxPrime>, defaults to 0");
    this->cx_prime_ = 0;
  }
  else
    this->cx_prime_ = this->sdf->GetValueDouble("CxPrime");

  if (!this->sdf->HasElement("Cx"))
  {
    ROS_INFO("Camera plugin missing <Cx>, defaults to 0");
    this->cx_= 0;
  }
  else
    this->cx_ = this->sdf->GetValueDouble("Cx");

  if (!this->sdf->HasElement("Cy"))
  {
    ROS_INFO("Camera plugin missing <Cy>, defaults to 0");
    this->cy_= 0;
  }
  else
    this->cy_ = this->sdf->GetValueDouble("Cy");

  if (!this->sdf->HasElement("focalLength"))
  {
    ROS_INFO("Camera plugin missing <focalLength>, defaults to 0");
    this->focal_length_= 0;
  }
  else
    this->focal_length_ = this->sdf->GetValueDouble("focalLength");

  if (!this->sdf->HasElement("hackBaseline"))
  {
    ROS_INFO("Camera plugin missing <hackBaseline>, defaults to 0");
    this->hack_baseline_= 0;
  }
  else
    this->hack_baseline_ = this->sdf->GetValueDouble("hackBaseline");

  if (!this->sdf->HasElement("distortionK1"))
  {
    ROS_INFO("Camera plugin missing <distortionK1>, defaults to 0");
    this->distortion_k1_= 0;
  }
  else
    this->distortion_k1_ = this->sdf->GetValueDouble("distortionK1");

  if (!this->sdf->HasElement("distortionK2"))
  {
    ROS_INFO("Camera plugin missing <distortionK2>, defaults to 0");
    this->distortion_k2_= 0;
  }
  else
    this->distortion_k2_ = this->sdf->GetValueDouble("distortionK2");

  if (!this->sdf->HasElement("distortionK3"))
  {
    ROS_INFO("Camera plugin missing <distortionK3>, defaults to 0");
    this->distortion_k3_= 0;
  }
  else
    this->distortion_k3_ = this->sdf->GetValueDouble("distortionK3");

  if (!this->sdf->HasElement("distortionT1"))
  {
    ROS_INFO("Camera plugin missing <distortionT1>, defaults to 0");
    this->distortion_t1_= 0;
  }
  else
    this->distortion_t1_ = this->sdf->GetValueDouble("distortionT1");

  if (!this->sdf->HasElement("distortionT2"))
  {
    ROS_INFO("Camera plugin missing <distortionT2>, defaults to 0");
    this->distortion_t2_= 0;
  }
  else
    this->distortion_t2_ = this->sdf->GetValueDouble("distortionT2");

  if ((this->distortion_k1_ != 0.0) || (this->distortion_k2_ != 0.0) ||
      (this->distortion_k3_ != 0.0) || (this->distortion_t1_ != 0.0) ||
      (this->distortion_t2_ != 0.0))
  {
    ROS_WARN("gazebo_ros_camera_ simulation does not support non-zero"
             " distortion parameters right now, your simulation maybe wrong.");
  }

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosCameraUtils::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::LoadThread()
{
  // Exit if no ROS
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_ +
    "/" + this->camera_name_);

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

#ifdef DYNAMIC_RECONFIGURE
  if (!this->camera_name_.empty())
  {
    dyn_srv_ =
      new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
      (*this->rosnode_);
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
      ::CallbackType f =
      boost::bind(&GazeboRosCameraUtils::configCallback, this, _1, _2);
    dyn_srv_->setCallback(f);
  }
  else
  {
    ROS_WARN("dynamic reconfigure is not enabled for this image topic [%s]"
             " becuase <cameraName> is not specified",
             this->image_topic_name_.c_str());
  }
#endif

  this->image_pub_ = this->itnode_->advertise(
    this->image_topic_name_, 1,
    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
    ros::VoidPtr(), &this->camera_queue_);

  // camera info publish rate will be synchronized to image sensor
  // publish rates.
  // If someone connects to camera_info, sensor will be activated
  // and camera_info will be published alongside image_raw with the
  // same timestamps.  This incurrs additional computational cost when
  // there are subscribers to camera_info, but better mimics behavior
  // of image_pipeline.
  ros::AdvertiseOptions cio =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->camera_info_topic_name_, 1,
    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
    ros::VoidPtr(), &this->camera_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(cio);

  /* disabling fov and rate setting for each camera
  ros::SubscribeOptions zoom_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_hfov", 1,
        boost::bind(&GazeboRosCameraUtils::SetHFOV, this, _1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraHFOVSubscriber_ = this->rosnode_->subscribe(zoom_so);

  ros::SubscribeOptions rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_update_rate", 1,
        boost::bind(&GazeboRosCameraUtils::SetUpdateRate, this, _1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraUpdateRateSubscriber_ = this->rosnode_->subscribe(rate_so);
  */

  this->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCameraUtils::SetHFOV(const std_msgs::Float64::ConstPtr& hfov)
{
  this->camera_->SetHFOV(hfov->data);
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosCameraUtils::SetUpdateRate(
  const std_msgs::Float64::ConstPtr& update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::ImageConnect()
{
  this->image_connect_count_++;
  // maintain for one more release for backwards compatibility
  // with pr2_gazebo_plugins
  this->parentSensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::ImageDisconnect()
{
  this->image_connect_count_--;
  // maintain for one more release for backwards compatibility with
  // pr2_gazebo_plugins
  if (this->image_connect_count_ <= 0)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;


  // sensor generation off by default
  this->parentSensor_->SetActive(false);

  // set buffer size
  if (this->format_ == "L8")
  {
    this->type_ = sensor_msgs::image_encodings::MONO8;
    this->skip_ = 1;
  }
  else if (this->format_ == "R8G8B8")
  {
    this->type_ = sensor_msgs::image_encodings::RGB8;
    this->skip_ = 3;
  }
  else if (this->format_ == "B8G8R8")
  {
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }
  else if (this->format_ == "BAYER_RGGB8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_BGGR8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GBRG8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GRBG8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  /// Compute camera_ parameters if set to 0
  if (this->cx_prime_ == 0)
    this->cx_prime_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cx_ == 0)
    this->cx_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cy_ == 0)
    this->cy_ = (static_cast<double>(this->height_) + 1.0) /2.0;


  double computed_focal_length =
    (static_cast<double>(this->width_)) /
    (2.0 * tan(this->camera_->GetHFOV().Radian() / 2.0));

  if (this->focal_length_ == 0)
  {
    this->focal_length_ = computed_focal_length;
  }
  else
  {
    // check against float precision
    if (!gazebo::math::equal(this->focal_length_, computed_focal_length))
    {
      ROS_WARN("The <focal_length>[%f] you have provided for camera_ [%s]"
               " is inconsistent with specified image_width [%d] and"
               " HFOV [%f].   Please double check to see that"
               " focal_length = width_ / (2.0 * tan(HFOV/2.0)),"
               " the explected focal_lengtth value is [%f],"
               " please update your camera_ model description accordingly.",
                this->focal_length_, this->parentSensor_->GetName().c_str(),
                this->width_, this->camera_->GetHFOV().Radian(),
                computed_focal_length);
    }
  }


  // start custom queue for camera_
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosCameraUtils::CameraQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src,
  common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src)
{
  /// don't bother if there are no subscribers
  if (this->image_pub_.getNumSubscribers() > 0)
  {
    boost::mutex::scoped_lock lock(this->lock_);

    // copy data into image
    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
    this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

    // copy from src to image_msg_
    fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(_src));

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PublishCameraInfo(common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PublishCameraInfo();
}

void GazeboRosCameraUtils::PublishCameraInfo()
{
  if (this->camera_info_pub_.getNumSubscribers() > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->camera_info_pub_);
      this->last_info_update_time_ = cur_time;
    }
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(
  ros::Publisher camera_info_publisher)
{
  sensor_msgs::CameraInfo camera_info_msg;
  // fill CameraInfo
  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;
  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.D.resize(5);
#endif
  camera_info_msg.D[0] = this->distortion_k1_;
  camera_info_msg.D[1] = this->distortion_k2_;
  camera_info_msg.D[2] = this->distortion_k3_;
  camera_info_msg.D[3] = this->distortion_t1_;
  camera_info_msg.D[4] = this->distortion_t2_;
  // original camera_ matrix
  camera_info_msg.K[0] = this->focal_length_;
  camera_info_msg.K[1] = 0.0;
  camera_info_msg.K[2] = this->cx_;
  camera_info_msg.K[3] = 0.0;
  camera_info_msg.K[4] = this->focal_length_;
  camera_info_msg.K[5] = this->cy_;
  camera_info_msg.K[6] = 0.0;
  camera_info_msg.K[7] = 0.0;
  camera_info_msg.K[8] = 1.0;
  // rectification
  camera_info_msg.R[0] = 1.0;
  camera_info_msg.R[1] = 0.0;
  camera_info_msg.R[2] = 0.0;
  camera_info_msg.R[3] = 0.0;
  camera_info_msg.R[4] = 1.0;
  camera_info_msg.R[5] = 0.0;
  camera_info_msg.R[6] = 0.0;
  camera_info_msg.R[7] = 0.0;
  camera_info_msg.R[8] = 1.0;
  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.P[0] = this->focal_length_;
  camera_info_msg.P[1] = 0.0;
  camera_info_msg.P[2] = this->cx_;
  camera_info_msg.P[3] = -this->focal_length_ * this->hack_baseline_;
  camera_info_msg.P[4] = 0.0;
  camera_info_msg.P[5] = this->focal_length_;
  camera_info_msg.P[6] = this->cy_;
  camera_info_msg.P[7] = 0.0;
  camera_info_msg.P[8] = 0.0;
  camera_info_msg.P[9] = 0.0;
  camera_info_msg.P[10] = 1.0;
  camera_info_msg.P[11] = 0.0;

  camera_info_publisher.publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode_->ok())
  {
    /// take care of callback queue
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
