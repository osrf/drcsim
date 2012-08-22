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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <ros_plugins/ros_joint_trajectory.h>

#include "tf/tf.h"

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
RosJointTrajectory::RosJointTrajectory()
{
  this->has_trajectory_ = false;
  this->joint_position_map.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosJointTrajectory::~RosJointTrajectory()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosJointTrajectory::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("serviceName"))
  {
    // default
    this->service_name_ = "set_joint_trajectory";
  }
  else
    this->service_name_ = _sdf->GetElement("serviceName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    // default
    this->topic_name_ = "set_joint_trajectory";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("joint trajectory plugin missing <updateRate>, defaults to 0.0 (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();

  if (!ros::isInitialized())
  {
    ROS_ERROR("ros should have been initialized gazebo_ros_api_plugins, please load the server plugin.");
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  //this->tf_reference_link_name_ = tf::resolve(prefix, this->reference_link_name_);

  if (this->topic_name_ != "")
  {
    ros::SubscribeOptions trajectory_so = ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>(
      this->topic_name_,100, boost::bind( &RosJointTrajectory::SetTrajectory,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(trajectory_so);
  }

  if (this->service_name_ != "")
  {
    ros::AdvertiseServiceOptions srv_aso = ros::AdvertiseServiceOptions::create<ros_plugins::SetJointTrajectory>(
        this->service_name_,boost::bind(&RosJointTrajectory::SetTrajectory,this,_1,_2),
        ros::VoidPtr(), &this->queue_);
    this->srv_ = this->rosnode_->advertiseService(srv_aso);
  }
  
  this->last_time_ = this->world_->GetSimTime();

  // start custom queue for joint trajectory plugin ros topics
  this->callback_queue_thread_ = boost::thread( boost::bind( &RosJointTrajectory::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&RosJointTrajectory::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// set joint trajectory
void RosJointTrajectory::SetTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
  boost::mutex::scoped_lock lock(this->update_mutex);

  ros_plugins::SetJointTrajectory::Request req;
  ros_plugins::SetJointTrajectory::Response res;
  // update the trajectory to play
  this->has_trajectory_ = true;

  this->reference_link_name_ = trajectory->header.frame_id;
  // do this every time a new trajectory is supplied, use header.frame_id as the reference_link_name_
  if (this->reference_link_name_ != "world" && this->reference_link_name_ != "/map" && this->reference_link_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->reference_link_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR("ros_joint_trajectory plugin: reference link [%s] does not exist, will not set model state and pose\n",
                this->reference_link_name_.c_str());
      return;
    }
  }
  if (this->reference_link_)
    ROS_DEBUG("will set model pose by setting pose of link [%s]",this->reference_link_->GetName().c_str());
  else
    ROS_DEBUG("will set model [%s] pose directly",this->model_->GetName().c_str());



  this->joint_position_map.clear();

  // copy joint configuration into a map
  for (unsigned int i = 0; i < _msg->joint_names().size(); i++)
    this->joint_position_map[_msg->joint_names().Get(i)] = _msg->joint_positions().Get(i);

  this->configuration_time = gazebo::common::Time(_msg->time().sec(), _msg->time().nsec());



}

bool RosJointTrajectory::SetTrajectory(const ros_plugins::SetJointTrajectory::Request& req, const ros_plugins::SetJointTrajectory::Response& res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void RosJointTrajectory::UpdateStates()
{
  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  this->lock.lock();

  if (this->has_trajectory_)
  {





      // set model configuration
      if (curTime >= this->configuration_time && !this->joint_position_map.empty())
      {
        gzdbg << "time [" << curTime << "] updating configuration [" << "..." << "]\n";
        // make the service call to pause gazebo
        bool is_paused = this->world->IsPaused();
        if (!is_paused) this->world->SetPaused(true);

        this->model->SetJointPositions(this->joint_position_map);

        // resume paused state before this call
        this->world->SetPaused(is_paused);
        this->joint_position_map.clear();
      }


      // play through the trajectory
      bool update_model_poses = false;
      math::Pose new_pose;

      // play through the poses in this trajectory
      for (; this->trajectory_index < this->trajectory_stamped.size(); ++this->trajectory_index )
      {

        const msgs::PoseStamped pose_stamped = this->trajectory_stamped.Get(this->trajectory_index);

        common::Time pose_time(pose_stamped.time().sec(), pose_stamped.time().nsec());

        if (curTime >= pose_time)
        {
          new_pose = math::Pose( math::Vector3( pose_stamped.pose().position().x(),
                                                pose_stamped.pose().position().y(),
                                                pose_stamped.pose().position().z() ),
                                 math::Quaternion( pose_stamped.pose().orientation().w(),
                                                   pose_stamped.pose().orientation().x(),
                                                   pose_stamped.pose().orientation().y(),
                                                   pose_stamped.pose().orientation().z() )
                               );
          update_model_poses = true;
        }
        else
          break;
      }

      if (update_model_poses)
      {
        gzdbg << "time [" << curTime << "] updating pose [" << new_pose << "]\n";
        this->model->SetLinkWorldPose( new_pose, "r_foot" );
      }

    // this->model->SetJointPositions(this->joint_position_map);

    // set model pose
    if (this->reference_link_)
    {
      // math::Pose    reference_pose = this->reference_link_->GetWorldPose();
      // math::Vector3 reference_vpos = this->reference_link_->GetWorldLinearVel();
      // math::Vector3 reference_veul = this->reference_link_->GetWorldAngularVel();
      // this->model->SetLinkWorldPose( new_pose, this->reference_link );
    }
    else
    {
      // this->model->SetWorldPose( new_pose );
    }

    bool trajectory_done = false;
    if (trajectory_done)
    {
      this->reference_link_.reset();
      this->has_trajectory_ = false;
    }
  }

  this->lock.unlock();

  // save last time stamp
  this->last_time_ = cur_time;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void RosJointTrajectory::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(RosJointTrajectory);

}
