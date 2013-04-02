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

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include "gazebo_ros_camera.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCamera::GazeboRosCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCamera::~GazeboRosCamera()
{
}

void GazeboRosCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();

  if (!this->parentSensor->IsActive())
  {
    if (this->image_connect_count_ > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->image_connect_count_ > 0)
    {
      common::Time cur_time = this->world_->GetSimTime();
      if (cur_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image);
        this->PublishCameraInfo();
        this->last_update_time_ = cur_time;
      }
    }
  }
}
}
