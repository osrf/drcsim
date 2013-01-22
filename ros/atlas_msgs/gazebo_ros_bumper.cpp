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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include <gazebo_plugins/gazebo_ros_bumper.h>

#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "physics/Contact.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/SensorTypes.hh"
#include "math/Pose.hh"
#include "math/Quaternion.hh"
#include "math/Vector3.hh"

#include "tf/tf.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBumper::GazeboRosBumper()
  : ContactPlugin()
{
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
  ContactPlugin::Load(_parent, _sdf);

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

  // Exit if no ROS
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api.so\n";
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(std::string(this->bumper_topic_name_),1);

  // Initialize
  // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
  // since most bodies are constructed on the fly
  //this->myFrame = NULL;
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread( 
      boost::bind( &GazeboRosBumper::ContactQueueThread,this ) );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosBumper::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBumper::UpdateChild()
{
  if (this->contact_pub_.getNumSubscribers() <= 0) 
    return;

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
  boost::mutex::scoped_lock sclock(*this->parentSensor->GetUpdateMutex());

  std::map<std::string, physics::Contact> contacts;
  math::Vector3 body1ContactForce, body2ContactForce;
  common::Time cur_time;

  cur_time = this->world->GetSimTime();

  // information are in inertial coordinates
  this->contact_state_msg_.header.frame_id = this->frame_name_;  
  this->contact_state_msg_.header.stamp.sec = cur_time.sec;
  this->contact_state_msg_.header.stamp.nsec = cur_time.nsec;;

  // set contact states size
  this->contact_state_msg_.states.clear();  // one contact_count per pair of geoms in contact (up to 64 contact points per pair of geoms)

  // For each collision that the sensor is monitoring
  for (unsigned int i=0; i < this->parentSensor->GetCollisionCount();i++)
  {
    int l = 0;
    std::string collisionName = this->parentSensor->GetCollisionName(i);
    contacts = this->parentSensor->GetContacts(collisionName);

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
    math::Quaternion rot, frame_rot;
    math::Vector3 pos, frame_pos;
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
    for (std::map<std::string, gazebo::physics::Contact>::iterator citer = 
         contacts.begin(); citer != contacts.end() ; citer++)
    {
      gazebo::physics::Contact contact = citer->second;

      // For each geom-geom contact
      unsigned int pts = contact.count;

      std::ostringstream stream;
      stream    << "touched!    i:" << l
        << "      my geom:" << contact.collision1->GetName()
        << "   other geom:" << contact.collision2->GetName()
        << "         time:" << contact.time
        << std::endl;

      gazebo_msgs::ContactState state;
      state.info = stream.str();
      state.collision1_name = contact.collision1->GetName();
      state.collision2_name = contact.collision2->GetName();

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

      for (unsigned int k=0; k < pts; k++)
      {
        // rotate into user specified frame. 
        // frame_rot is identity if world is used.
        math::Vector3 force = frame_rot.RotateVectorReverse(
            math::Vector3(contact.forces[k].body1Force.x,
              contact.forces[k].body1Force.y,
              contact.forces[k].body1Force.z));
        math::Vector3 torque = frame_rot.RotateVectorReverse(
            math::Vector3(contact.forces[k].body1Torque.x,
              contact.forces[k].body1Torque.y,
              contact.forces[k].body1Torque.z));

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
        gazebo::math::Vector3 contact_position;
        contact_position = contact.positions[k] - frame_pos;
        contact_position = frame_rot.RotateVectorReverse(contact_position);
        geometry_msgs::Vector3 tmp;
        tmp.x = contact_position.x;
        tmp.y = contact_position.y;
        tmp.z = contact_position.z;
        state.contact_positions.push_back(tmp);

        // rotate normal into user specified frame. 
        // frame_rot is identity if world is used.
        math::Vector3 normal = frame_rot.RotateVectorReverse(
            math::Vector3(contact.normals[k].x,
              contact.normals[k].y,
              contact.normals[k].z));
        // set contact normals
        geometry_msgs::Vector3 contact_normal;
        contact_normal.x = normal.x;
        contact_normal.y = normal.y;
        contact_normal.z = normal.z;
        state.contact_normals.push_back(contact_normal);

        // set contact depth, interpenetration
        state.depths.push_back(contact.depths[k]);
      }
      state.total_wrench = total_wrench;
      this->contact_state_msg_.states.push_back(state);
    }
  }

  this->contact_pub_.publish(this->contact_state_msg_);
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
