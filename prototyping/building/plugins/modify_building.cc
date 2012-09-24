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
  class ModifyBuilding : public ModelPlugin
  {

    public: ModifyBuilding()
    {
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
          boost::bind(&ModifyBuilding::OnUpdate, this));
      gzdbg << "plugin model name: " << this->model->GetName() << "\n";


      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->GetName());
      this->sub = this->node->Subscribe("/gazebo/modify_building", &ModifyBuilding::OnModifyBuilding, this);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      common::Time curTime  = this->world->GetSimTime();
    }



    public: void OnModifyBuilding( const boost::shared_ptr<msgs::Test const> &_msg)
    {
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModifyBuilding)
}
