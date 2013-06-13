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

#include "sensors/Sensor.hh"
#include "sensors/MultiCameraSensor.hh"
#include "sensors/SensorTypes.hh"

#include "gazebo_ros_multicamera.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMultiCamera::GazeboRosMultiCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMultiCamera::~GazeboRosMultiCamera()
{
}

void GazeboRosMultiCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  for (unsigned i = 0; i < this->camera.size(); ++i)
  {
    GazeboRosCameraUtils* util = new GazeboRosCameraUtils();
    util->parentSensor_ = this->parentSensor;
    util->width_   = this->width[i];
    util->height_  = this->height[i];
    util->depth_   = this->depth[i];
    util->format_  = this->format[i];
    util->camera_  = this->camera[i];
    // Set up a shared connection counter
    this->imageConnectCount = 0;
    util->image_connect_count_ = boost::shared_ptr<int>(&this->imageConnectCount);
    util->image_connect_count_lock_ =
      boost::shared_ptr<boost::mutex>(&this->imageConnectCountLock);
    this->wasActive = false;
    util->was_active_ =
      boost::shared_ptr<bool>(&this->wasActive);
    if (this->camera[i]->GetName().find("left") != std::string::npos)
    {
      // FIXME: hardcoded, left hack_baseline_ 0
      util->Load(_parent, _sdf, "/left", 0.0);
    }
    else if (this->camera[i]->GetName().find("right") != std::string::npos)
    {
      double hackBaseline = 0.0;
      if (_sdf->HasElement("hackBaseline"))
        hackBaseline = _sdf->GetValueDouble("hackBaseline");
      util->Load(_parent, _sdf, "/right", hackBaseline);
    }
    this->utils.push_back(util);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMultiCamera::OnNewFrameLeft(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  GazeboRosCameraUtils* util = this->utils[0];
  util->sensor_update_time_ = util->parentSensor_->GetLastUpdateTime();

  if (util->parentSensor_->IsActive())
  {
    common::Time cur_time = util->world_->GetSimTime();
    if (cur_time - util->last_update_time_ >= util->update_period_)
    {
      util->PutCameraData(_image);
      util->PublishCameraInfo();
      util->last_update_time_ = cur_time;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMultiCamera::OnNewFrameRight(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  GazeboRosCameraUtils* util = this->utils[1];
  util->sensor_update_time_ = util->parentSensor_->GetLastUpdateTime();

  if (util->parentSensor_->IsActive())
  {
    common::Time cur_time = util->world_->GetSimTime();
    if (cur_time - util->last_update_time_ >= util->update_period_)
    {
      util->PutCameraData(_image);
      util->PublishCameraInfo();
      util->last_update_time_ = cur_time;
    }
  }
}
}
