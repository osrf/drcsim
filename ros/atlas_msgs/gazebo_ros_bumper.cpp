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

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sdf/interface/SDF.hh>
#include <gazebo/sdf/interface/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <tf/tf.h>

#include "gazebo_ros_bumper.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBumper::GazeboRosBumper()
  : ContactPlugin()
{
  this->contact_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBumper::~GazeboRosBumper()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBumper::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = boost::shared_dynamic_cast<gazebo::sensors::ContactSensor>(_parent);

  std::string world_name = _parent->GetWorldName();
  this->world_ = physics::get_world(world_name);

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  // "publishing contact/collisions to this topic name: " << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->GetElement("bumperTopicName"))
    this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->GetValueString();

  // "transform contact/collisions pose, forces to this body (link) name: " << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",
        ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  ros::AdvertiseOptions ao = 
    ros::AdvertiseOptions::create<atlas_msgs::ContactsState>(
        std::string(this->bumper_topic_name_),1,
        boost::bind( &GazeboRosBumper::ContactConnect,this),
        boost::bind( &GazeboRosBumper::ContactDisconnect,this), 
        ros::VoidPtr(), &this->contact_queue_);
  this->contact_pub_ = this->rosnode_->advertise(ao);

  // Initialize
  // preset myFrame to NULL, will search for the body with matching name in OnUpdate()
  // since most bodies are constructed on the fly
  //this->myFrame = NULL;
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread( 
      boost::bind( &GazeboRosBumper::ContactQueueThread,this ) );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  ContactPlugin::Load(_parent, _sdf);
  this->update_connection_ = this->parent_->ConnectUpdated(
    boost::bind(&GazeboRosBumper::OnUpdate, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosBumper::ContactConnect()
{
  this->contact_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosBumper::ContactDisconnect()
{
  this->contact_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBumper::OnUpdate()
{
  if (this->contact_connect_count_ <= 0) 
    return;

  gazebo::msgs::Contacts contacts;
  contacts = this->parent_->GetContacts();

  for (int i = 0; i < contacts.contact_size(); ++i)
  {

    /*
    /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
    if (this->myFrame == NULL && this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "map")
    {
      // lock in case a model is being spawned
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      // look through all models in the world, search for body name that matches frameName
      std::vector<Model*> all_models = World::Instance()->GetModels();
      for (std::vector<Model*>::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
      {
        if (*iter) this->myFrame = dynamic_cast<Body*>((*iter)->GetBody(this->frame_name_));
        if (this->myFrame) break;
      }

      // not found
      if (this->myFrame == NULL)
      {
        ROS_DEBUG("gazebo_ros_bumper plugin: frameName: %s does not exist yet, will not publish\n",this->frame_name_.c_str());
        return;
      }
    }
   */
    //boost::mutex::scoped_lock sclock(*this->parentSensor->GetUpdateMutex());

    // information are in inertial coordinates
    this->contact_state_msg_.header.frame_id = this->frame_name_;  
    this->contact_state_msg_.header.stamp.sec = contacts.time().sec();
    this->contact_state_msg_.header.stamp.nsec = contacts.time().nsec();

    // set contact states size
    this->contact_state_msg_.states.clear();  // one contact_count per pair of geoms in contact (up to 64 contact points per pair of geoms)

    {
      // populate header
      //if (cur_body)
      //  this->contactMsg.header.frame_id = cur_body->GetName();  // @todo: transform results to the link name
      
      // get contact array sizes for this contact param.  
      // each contact param has num_contact_count contact pairs,
      // each contact geom pair has up to 64 contact points
      //std::vector<physics::Collision*> contact_geoms;

      // get reference frame (body(link)) pose and subtract from it to get 
      // relative force, torque, position and normal vectors
      math::Pose pose, frame_pose;
      math::Quaternion frame_rot;
      math::Vector3 frame_pos;
      /*
      if (this->myFrame)
      {
        frame_pose = this->myFrame->GetWorldPose();//-this->myBody->GetCoMPose();
        frame_pos = frame_pose.pos;
        frame_rot = frame_pose.rot;
      }
      else
      */
      {
        // no specific frames specified, use identity pose, keeping 
        // relative frame at inertial origin
        frame_pos = math::Vector3(0,0,0);
        frame_rot = math::Quaternion(1,0,0,0); // gazebo u,x,y,z == identity
        frame_pose = math::Pose(frame_pos, frame_rot);
      }

      // For each collision contact 
      {
        // For each geom-geom contact

        atlas_msgs::ContactState state;
        state.collision1_name = contacts.contact(i).collision1();
        state.collision2_name = contacts.contact(i).collision2();
        std::ostringstream stream;
        stream    << "touched!    i:" << i
          << "      my geom:" << state.collision1_name
          << "   other geom:" << state.collision2_name
          << "         time:" << gazebo::common::Time(contacts.time().sec(), contacts.time().nsec())
          << std::endl;
        state.info = stream.str();

        state.wrenches.clear();
        state.contact_positions.clear();
        state.contact_normals.clear();
        state.depths.clear();

        // sum up all wrenches for each DOF
        geometry_msgs::Wrench total_wrench;
        total_wrench.force.x = 0;
        total_wrench.force.y = 0;
        total_wrench.force.z = 0;
        total_wrench.torque.x = 0;
        total_wrench.torque.y = 0;
        total_wrench.torque.z = 0;

        // For each collision that the sensor is monitoring
        for (int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
          // rotate into user specified frame. 
          // frame_rot is identity if world is used.
          math::Vector3 force = frame_rot.RotateVectorReverse(
              math::Vector3(contacts.contact(i).wrench(j).body_1_force().x(),
                            contacts.contact(i).wrench(j).body_1_force().y(),
                            contacts.contact(i).wrench(j).body_1_force().z()));
          math::Vector3 torque = frame_rot.RotateVectorReverse(
              math::Vector3(contacts.contact(i).wrench(j).body_1_torque().x(),
                            contacts.contact(i).wrench(j).body_1_torque().y(),
                            contacts.contact(i).wrench(j).body_1_torque().z()));

          // set wrenches
          geometry_msgs::Wrench wrench;
          wrench.force.x  = force.x;
          wrench.force.y  = force.y;
          wrench.force.z  = force.z;
          wrench.torque.x = torque.x;
          wrench.torque.y = torque.y;
          wrench.torque.z = torque.z;
          state.wrenches.push_back(wrench);
          total_wrench.force.x  += wrench.force.x ;
          total_wrench.force.y  += wrench.force.y ;
          total_wrench.force.z  += wrench.force.z ;
          total_wrench.torque.x += wrench.torque.x;
          total_wrench.torque.y += wrench.torque.y;
          total_wrench.torque.z += wrench.torque.z;

          // transform contact positions into relative frame
          // set contact positions
          gazebo::math::Vector3 contact_position(contacts.contact(i).position(j).x(),
                                                 contacts.contact(i).position(j).y(),
                                                 contacts.contact(i).position(j).z());
          contact_position -= frame_pos;
          contact_position = frame_rot.RotateVectorReverse(contact_position);
          geometry_msgs::Vector3 tmp;
          tmp.x = contact_position.x;
          tmp.y = contact_position.y;
          tmp.z = contact_position.z;
          state.contact_positions.push_back(tmp);

          // rotate normal into user specified frame. 
          // frame_rot is identity if world is used.
          math::Vector3 normal = frame_rot.RotateVectorReverse(
              math::Vector3(contacts.contact(i).normal(j).x(),
                            contacts.contact(i).normal(j).y(),
                            contacts.contact(i).normal(j).z()));
          // set contact normals
          geometry_msgs::Vector3 contact_normal;
          contact_normal.x = normal.x;
          contact_normal.y = normal.y;
          contact_normal.z = normal.z;
          state.contact_normals.push_back(contact_normal);

          // set contact depth, interpenetration
          state.depths.push_back(contacts.contact(i).depth(j));
        }
        state.total_wrench = total_wrench;
        this->contact_state_msg_.states.push_back(state);
      }
    }

    this->contact_pub_.publish(this->contact_state_msg_);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosBumper::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)

}
