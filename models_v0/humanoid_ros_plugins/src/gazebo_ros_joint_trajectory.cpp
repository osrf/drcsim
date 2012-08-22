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

#include <humanoid_ros_plugins/gazebo_ros_joint_trajectory.h>

#include "tf/tf.h"

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosJointTrajectory::GazeboRosJointTrajectory()
{
  this->has_trajectory_ = false;
  this->trajectory_index = 0;
  this->joint_trajectory_.points.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosJointTrajectory::~GazeboRosJointTrajectory()
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
void GazeboRosJointTrajectory::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
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
      this->topic_name_,100, boost::bind( &GazeboRosJointTrajectory::SetTrajectory,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(trajectory_so);
  }

  if (this->service_name_ != "")
  {
    ros::AdvertiseServiceOptions srv_aso = ros::AdvertiseServiceOptions::create<humanoid_ros_plugins::SetJointTrajectory>(
        this->service_name_,boost::bind(&GazeboRosJointTrajectory::SetTrajectory,this,_1,_2),
        ros::VoidPtr(), &this->queue_);
    this->srv_ = this->rosnode_->advertiseService(srv_aso);
  }
  
  this->last_time_ = this->world_->GetSimTime();

  // start custom queue for joint trajectory plugin ros topics
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosJointTrajectory::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosJointTrajectory::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// set joint trajectory
void GazeboRosJointTrajectory::SetTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
  humanoid_ros_plugins::SetJointTrajectory::Request req;
  humanoid_ros_plugins::SetJointTrajectory::Response res;
  // copy topic into request of service call
  req.joint_trajectory = *trajectory;
  if (!this->SetTrajectory(req,res))
    ROS_WARN("setting joint trajectory via topic returned failure");
}

bool GazeboRosJointTrajectory::SetTrajectory(const humanoid_ros_plugins::SetJointTrajectory::Request& req, const humanoid_ros_plugins::SetJointTrajectory::Response& res)
{
  boost::mutex::scoped_lock lock(this->update_mutex);

  this->reference_link_name_ = req.joint_trajectory.header.frame_id;
  // do this every time a new joint_trajectory is supplied, use header.frame_id as the reference_link_name_
  if (this->reference_link_name_ != "world" && this->reference_link_name_ != "/map" && this->reference_link_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->reference_link_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR("ros_joint_trajectory plugin specified a reference link [%s] that does not exist, aborting.\n",
                this->reference_link_name_.c_str());
      return false;
    }
  }
  if (this->reference_link_)
    ROS_DEBUG("will set model pose by setting pose of link [%s]",this->reference_link_->GetName().c_str());
  else
    ROS_DEBUG("will set model [%s] pose directly",this->model_->GetName().c_str());

  // copy joint configuration into a map
  this->joint_trajectory_ = req.joint_trajectory;

/*
  unsigned int trajectory_size = this->joint_trajectory.points.size();
  for (unsigned int i = 0; i < chain_size; i++)
  {
    for (unsigned int j = 0; j < trajectory_size; j++)
    {
    }
  }
*/


  // trajectory start time
  this->trajectory_start = gazebo::common::Time(req.joint_trajectory.header.stamp.sec,
                                                req.joint_trajectory.header.stamp.nsec);

  // update the joint_trajectory to play
  this->has_trajectory_ = true;
  // reset trajectory_index to beginning of new trajectory
  this->trajectory_index = 0;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void GazeboRosJointTrajectory::UpdateStates()
{
  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  {
    boost::mutex::scoped_lock lock(this->update_mutex);

    if (this->has_trajectory_)
    {

      // roll out trajectory via set model configuration
      if (cur_time >= this->trajectory_start)
      {
        if (this->trajectory_index < this->joint_trajectory_.points.size())
        {
          gzdbg << "time [" << cur_time << "] updating configuration [" << this->trajectory_index << "]\n";
          // make the service call to pause gazebo
          bool is_paused = this->world_->IsPaused();
          if (!is_paused) this->world_->SetPaused(true);

          // get model configuration
          std::map<std::string, double> joint_position_map;

          unsigned int chain_size = this->joint_trajectory_.joint_names.size();
          for (unsigned int i = 0; i < chain_size; ++i)
          {
            joint_position_map[this->joint_trajectory_.joint_names[i]] = 
              this->joint_trajectory_.points[this->trajectory_index].positions[i];
          }
          this->model_->SetJointPositions(joint_position_map);

          this->world_->SetPaused(is_paused); // resume original pause-state
          this->trajectory_index++;
          gazebo::common::Time duration(this->joint_trajectory_.points[this->trajectory_index].time_from_start.sec,
                                        this->joint_trajectory_.points[this->trajectory_index].time_from_start.nsec);
          this->trajectory_start += duration;
        }
      }



      // set model pose
      if (this->reference_link_)
      {
        // math::Pose    reference_pose = this->reference_link_->GetWorldPose();
        // math::Vector3 reference_vpos = this->reference_link_->GetWorldLinearVel();
        // math::Vector3 reference_veul = this->reference_link_->GetWorldAngularVel();
        // this->model_->SetLinkWorldPose( new_pose, this->reference_link );
      }
      else
      {
        // this->model_->SetWorldPose( new_pose );
      }

      bool trajectory_done = false;
      if (trajectory_done)
      {
        this->reference_link_.reset();
        this->has_trajectory_ = false;
      }
    }
  } // mutex lock

  // save last time stamp
  this->last_time_ = cur_time;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosJointTrajectory::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointTrajectory);

}
