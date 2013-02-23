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

#include "sensor_msgs/Imu.h"

#include "MultiSenseSLPlugin.h"

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MultiSenseSL)

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::MultiSenseSL()
{
  /// \todo: hardcoded for now, make them into plugin parameters
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1., -1., 10.0, -10.0);
  this->spindleOn = true;
  this->spindleSpeed = 0;
  this->spindleMaxRPM = 50.0;
  this->spindleMinRPM = 0;
  this->multiCameraFrameRate = 25.0;
  this->multiCameraExposureTime = 0.001;
  this->multiCameraGain = 1.0;
  // the parent link of the imu_sensor ends up being pelvis after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_senosr block.
  this->imuLinkName = "head";
}

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::~MultiSenseSL()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->atlasModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;

  ROS_INFO("Loading MultiSense ROS node.");

  this->lastTime = this->world->GetSimTime();

  // Get imu link
  this->imuLink = this->atlasModel->GetLink(this->imuLinkName);
  if (!this->imuLink)
    gzerr << this->imuLinkName << " not found\n";

  // Get sensors
  this->imuSensor =
    boost::shared_dynamic_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->atlasModel->GetScopedName()
        + "::head::"
        "head_imu_sensor"));
  if (!this->imuSensor)
    gzerr << "head_imu_sensor not found\n" << "\n";

  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)
  this->spindleLink = this->atlasModel->GetLink("atlas::hokuyo_link");
  if (!this->spindleLink)
  {
    gzerr << "spindle link not found, plugin will stop loading\n";
    return;
  }

  this->spindleJoint = this->atlasModel->GetJoint("atlas::hokuyo_joint");
  if (!this->spindleJoint)
  {
    gzerr << "spindle joint not found, plugin will stop loading\n";
    return;
  }

  // publish joint states for spindle joint
  this->jointStates.name.resize(1);
  this->jointStates.position.resize(1);
  this->jointStates.velocity.resize(1);
  this->jointStates.effort.resize(1);

  // sensors::Sensor_V s = sensors::SensorManager::Instance()->GetSensors();
  // for (sensors::Sensor_V::iterator siter = s.begin();
  //                                  siter != s.end(); ++siter)
  //   gzerr << (*siter)->GetName() << "\n";

  this->multiCameraSensor =
    boost::shared_dynamic_cast<sensors::MultiCameraSensor>(
    sensors::SensorManager::Instance()->GetSensor("stereo_camera"));
  if (!this->multiCameraSensor)
    gzerr << "multicamera sensor not found\n";

  this->laserSensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("head_hokuyo_sensor"));
  if (!this->laserSensor)
    gzerr << "laser sensor not found\n";

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&MultiSenseSL::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::LoadThread()
{
  // create ros node
  this->rosnode_ = new ros::NodeHandle("");

  // ros publication
  this->pubJointStates = this->rosnode_->advertise<sensor_msgs::JointState>(
    "multisense_sl/joint_states", 10);

  // publish imu data
  this->pubImu =
    this->rosnode_->advertise<sensor_msgs::Imu>(
      "multisense_sl/imu", 10);

  // ros subscription
  ros::SubscribeOptions set_spindle_speed_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_spindle_speed", 100,
    boost::bind(static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetSpindleSpeed), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_sub_ =
    this->rosnode_->subscribe(set_spindle_speed_so);

  /* not implemented, not supported
  ros::SubscribeOptions set_spindle_state_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
    "multisense_sl/set_spindle_state", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Bool::ConstPtr&)>(
        &MultiSenseSL::SetSpindleState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_sub_ =
    this->rosnode_->subscribe(set_spindle_state_so);

  ros::SubscribeOptions set_multi_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_frame_rate", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraFrameRate),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so);

  ros::SubscribeOptions set_multi_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_multi_camera_exposure_time_so);

  ros::SubscribeOptions set_multi_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    "multisense_sl/set_camera_gain", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_gain_sub_ =
    this->rosnode_->subscribe(set_multi_camera_gain_so);
  */

  /// \todo: waiting for gen_srv to be implemented (issue #37)
  /* Advertise services on the custom queue
  std::string set_spindle_speed_service_name(
    "multisense_sl/set_spindle_speed");
  ros::AdvertiseServiceOptions set_spindle_speed_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_speed_service_name,
      boost::bind(&MultiSenseSL::SetSpindleSpeed,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_service_ =
    this->rosnode_->advertiseService(set_spindle_speed_aso);

  std::string set_spindle_state_service_name(
    "multisense_sl/set_spindle_state");
  ros::AdvertiseServiceOptions set_spindle_state_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_state_service_name,
      boost::bind(&MultiSenseSL::SetSpindleState,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_service_ =
    this->rosnode_->advertiseService(set_spindle_state_aso);
  */

  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&MultiSenseSL::QueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&MultiSenseSL::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  // get imu data from imu link
  if (this->imuSensor)
  {
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = this->imuLinkName;
    imuMsg.header.stamp = ros::Time(curTime.Double());

    // compute angular rates
    {
      math::Vector3 wLocal = this->imuSensor->GetAngularVelocity();
      imuMsg.angular_velocity.x = wLocal.x;
      imuMsg.angular_velocity.y = wLocal.y;
      imuMsg.angular_velocity.z = wLocal.z;
    }

    // compute acceleration
    {
      math::Vector3 accel = this->imuSensor->GetLinearAcceleration();
      imuMsg.linear_acceleration.x = accel.x;
      imuMsg.linear_acceleration.y = accel.y;
      imuMsg.linear_acceleration.z = accel.z;
    }

    // compute orientation
    {
      math::Quaternion imuRot =
        this->imuSensor->GetOrientation();
      imuMsg.orientation.x = imuRot.x;
      imuMsg.orientation.y = imuRot.y;
      imuMsg.orientation.z = imuRot.z;
      imuMsg.orientation.w = imuRot.w;
    }

    this->pubImu.publish(imuMsg);
  }

  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->jointStates.name[0] = this->spindleJoint->GetName();
    this->jointStates.position[0] = this->spindleJoint->GetAngle(0).Radian();
    this->jointStates.velocity[0] = this->spindleJoint->GetVelocity(0);
    this->jointStates.effort[0] = 0;

    if (this->spindleOn)
    {
      // PID control (velocity) spindle
      double spindleError = this->spindleJoint->GetVelocity(0)
                          - this->spindleSpeed;
      double spindleCmd = this->spindlePID.Update(spindleError, dt);
      this->spindleJoint->SetForce(0, spindleCmd);

      this->jointStates.effort[0] = spindleCmd;

      this->lastTime = curTime;
    }
    else
    {
      this->spindlePID.Reset();
    }
    this->pubJointStates.publish(this->jointStates);
  }
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleSpeed(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleState(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  this->spindleSpeed = static_cast<double>(_msg->data);
  if (this->spindleSpeed > this->spindleMaxRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMaxRPM * 2.0*M_PI / 60.0;
  else if (this->spindleSpeed < this->spindleMinRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMinRPM * 2.0*M_PI / 60.0;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleState(const std_msgs::Bool::ConstPtr &_msg)
{
  this->spindleOn = static_cast<double>(_msg->data);
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraFrameRate = static_cast<double>(_msg->data);
  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraExposureTime(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraExposureTime = static_cast<double>(_msg->data);
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraGain = static_cast<double>(_msg->data);
  gzwarn << "setting camera gain in sim not implemented\n";
}
}
