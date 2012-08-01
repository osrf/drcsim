/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "physics/physics.h"
#include "transport/Node.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
  class MoveRagdoll : public ModelPlugin
  {

    public: MoveRagdoll()
    {
      this->trajectory_stamped.Clear();
      this->joint_position_map.clear();
    }

    public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
      // Get then name of the parent model
      //std::string modelName = _sdf->GetParent()->GetValueString("name");

      // Get the world name.
      this->world = _parent->GetWorld();

      // Get a pointer to the model
      //this->model = this->world->GetModel(modelName);
      this->model = _parent;

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&MoveRagdoll::OnUpdate, this));
      gzdbg << "plugin model name: " << this->model->GetName() << "\n";


      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->GetName());
      this->modelPoseSub = this->node->Subscribe("/gazebo/model_poses", &MoveRagdoll::OnModelPose, this);
      this->modelConfigurationSub = this->node->Subscribe("/gazebo/model_configuration", &MoveRagdoll::OnModelConfiguration, this);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      boost::mutex::scoped_lock lock(this->update_mutex);

      common::Time curTime  = this->world->GetSimTime();

      // hack to keep head up
      // physics::LinkPtr head = this->model->GetLink("head");
      // head->SetForce(math::Vector3(0,0,1));


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
      for (; this->trajectory_index < this->trajectory_stamped.size(); ++this->trajectory_index ) {

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
        this->model->SetLinkWorldPose( new_pose, "l_foot" );
      }


    }



    public: void OnModelPose( const boost::shared_ptr<msgs::PoseTrajectory const> &_msg)
    {
      boost::mutex::scoped_lock lock(this->update_mutex);
      // get name and id
      this->trajectory_name = _msg->name();
      this->trajectory_id = _msg->id();
      this->trajectory_stamped = _msg->pose_stamped();

      // reset trajectory_index to beginning of new trajectory
      this->trajectory_index = 0;
    }

    public: void OnModelConfiguration( const boost::shared_ptr<msgs::ModelConfiguration const> &_msg)
    {
      boost::mutex::scoped_lock lock(this->update_mutex);

      this->joint_position_map.clear();

      // copy joint configuration into a map
      for (unsigned int i = 0; i < _msg->joint_names().size(); i++)
        this->joint_position_map[_msg->joint_names().Get(i)] = _msg->joint_positions().Get(i);

      this->configuration_time = gazebo::common::Time(_msg->time().sec(), _msg->time().nsec());
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr modelPoseSub;
    private: transport::SubscriberPtr modelConfigurationSub;
    private: physics::WorldPtr world;

    // trajectory
    private: std::string trajectory_name;
    private: unsigned int trajectory_id;
    private: unsigned int trajectory_index;
    ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped > trajectory_stamped;
    private: std::map<std::string, double> joint_position_map;
    private: common::Time configuration_time;
    boost::mutex update_mutex;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveRagdoll)
}
