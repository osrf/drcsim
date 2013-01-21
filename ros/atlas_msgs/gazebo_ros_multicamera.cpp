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
 @mainpage
   Desc: GazeboRosMultiCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosMultiCamera plugin broadcasts ROS Image messages
 */

#include "gazebo_ros_multicamera.h"

#include "sensors/Sensor.hh"
#include "sensors/MultiCameraSensor.hh"
#include "sensors/SensorTypes.hh"

namespace gazebo
{

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

void GazeboRosMultiCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
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
    util->Load(_parent, _sdf);
    this->utils.push_back(util);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMultiCamera::OnNewFrame0(const unsigned char *_image, 
    unsigned int _width, unsigned int _height, unsigned int _depth, 
    const std::string &_format)
{
  GazeboRosCameraUtils* util = this->utils[0];
  util->sensor_update_time_ = util->parentSensor_->GetLastUpdateTime();

  if (!util->parentSensor_->IsActive())
  {
    if (util->image_connect_count_ > 0)
      // activate first so there's chance for sensor to run 1 frame after activate
      util->parentSensor_->SetActive(true);
  }
  else
  {
    if (util->image_connect_count_ > 0)
    {
      common::Time cur_time = util->world_->GetSimTime();
      if (cur_time - util->last_update_time_ >= util->update_period_)
      {
        util->PutCameraData(_image);
        util->last_update_time_ = cur_time;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMultiCamera::OnNewFrame1(const unsigned char *_image, 
    unsigned int _width, unsigned int _height, unsigned int _depth, 
    const std::string &_format)
{
  GazeboRosCameraUtils* util = this->utils[1];
  util->sensor_update_time_ = util->parentSensor_->GetLastUpdateTime();

  if (!util->parentSensor_->IsActive())
  {
    if (util->image_connect_count_ > 0)
      // activate first so there's chance for sensor to run 1 frame after activate
      util->parentSensor_->SetActive(true);
  }
  else
  {
    if (util->image_connect_count_ > 0)
    {
      common::Time cur_time = util->world_->GetSimTime();
      if (cur_time - util->last_update_time_ >= util->update_period_)
      {
        util->PutCameraData(_image);
        util->last_update_time_ = cur_time;
      }
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiCamera)

}
