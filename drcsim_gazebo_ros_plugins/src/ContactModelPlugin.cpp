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

#include <gazebo/physics/ContactManager.hh>
#include <gazebo/transport/transport.hh>
#include "drcsim_gazebo_ros_plugins/ContactModelPlugin.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ContactModelPlugin)

/////////////////////////////////////////////////
ContactModelPlugin::ContactModelPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
ContactModelPlugin::~ContactModelPlugin()
{
  this->contactSub.reset();
  this->contactsPub.reset();
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->collisions.clear();
}

/////////////////////////////////////////////////
void ContactModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();
  this->sdf = _sdf;

  std::string collisionName;
  std::string collisionScopedName;

  if (_sdf->HasElement("contact"))
  {
    sdf::ElementPtr collisionElem =
        _sdf->GetElement("contact")->GetElement("collision");
    // Get all the collision elements
    while (collisionElem)
    {
      // get collision name
      collisionName = collisionElem->Get<std::string>();
      this->collisions.insert(_model->GetName() + "::" + collisionName);
      collisionElem = collisionElem->GetNextElement("collision");
    }
  }
}

//////////////////////////////////////////////////
void ContactModelPlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->world->GetName());

  // Create a publisher for the contact information.
  if (this->sdf->HasElement("contact") &&
      this->sdf->GetElement("contact")->HasElement("topic") &&
      this->sdf->GetElement("contact")->Get<std::string>("topic")
      != "__default_topic__")
  {
    // This will create a topic based on the name specified in SDF.
    this->contactsPub = this->node->Advertise<msgs::Contacts>(
        this->sdf->GetElement("contact")->Get<std::string>("topic"));
  }
  else
  {
    // This will create a topic based on the name of the parent and the
    // name of the sensor.
    static int contactNum = 0;
    std::string topicName = "~/";
    topicName += this->model->GetName() + "/contact_" +
        boost::lexical_cast<std::string>(contactNum++);
    boost::replace_all(topicName, "::", "/");
    this->filterTopicName = this->model->GetName() + "_tactile_" +
        boost::lexical_cast<std::string>(contactNum) + "_filter";
    boost::replace_all(this->filterTopicName, "::", "/");
    this->contactsPub = this->node->Advertise<msgs::Contacts>(topicName);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ContactModelPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void ContactModelPlugin::OnUpdate()
{
  // only subscribe when needed
  if (this->contactsPub && this->contactsPub->HasConnections())
  {
    if (!this->contactSub)
    {
      if (!this->collisions.empty())
      {
        // request the contact manager to publish messages to a custom topic for
        // this sensor
        physics::ContactManager *mgr =
            this->world->GetPhysicsEngine()->GetContactManager();
        std::vector<std::string> collisionNames;
        std::copy(this->collisions.begin(), this->collisions.end(),
            std::back_inserter(collisionNames));
        std::string topic = mgr->CreateFilter(this->filterTopicName,
            collisionNames);
        this->contactSub = this->node->Subscribe(topic,
            &ContactModelPlugin::OnContacts, this);
      }
    }
  }
  else
  {
    return;
  }

  boost::mutex::scoped_lock lock(this->mutex);
  boost::unordered_set<std::string>::iterator collIter;
  std::string collision1;

  // Don't do anything if there is no new data to process.
  if (this->incomingContacts.empty())
    return;

  // Clear the outgoing contact message.
  this->contactsMsg.clear_contact();

  // Iterate over all the contact messages
  for (ContactMsgs_L::iterator iter = this->incomingContacts.begin();
      iter != this->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      collision1 = (*iter)->contact(i).collision1();

      // Try to find the first collision's name
      collIter = this->collisions.find(collision1);

      // If unable to find the first collision's name, try the second
      if (collIter == this->collisions.end())
      {
        collision1 = (*iter)->contact(i).collision2();
        collIter = this->collisions.find(collision1);
      }

      // If this model is monitoring one of the collision's in the
      // contact, then add the contact to our outgoing message.
      if (collIter != this->collisions.end())
      {
        int count = (*iter)->contact(i).position_size();

        // Check to see if the contact arrays all have the same size.
        if (count != (*iter)->contact(i).normal_size() ||
            count != (*iter)->contact(i).wrench_size() ||
            count != (*iter)->contact(i).depth_size())
        {
          gzerr << "Contact message has invalid array sizes\n";
          continue;
        }
        // Copy the contact message.
        msgs::Contact *contactMsg = this->contactsMsg.add_contact();
        contactMsg->CopyFrom((*iter)->contact(i));
      }
    }
  }

  // Clear the incoming contact list.
  this->incomingContacts.clear();

  // Generate an outgoing message only if someone is listening.
  if (this->contactsPub && this->contactsPub->HasConnections())
  {
    msgs::Set(this->contactsMsg.mutable_time(), this->world->GetSimTime());
    this->contactsPub->Publish(this->contactsMsg);
  }
}

//////////////////////////////////////////////////
void ContactModelPlugin::OnContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Store the contacts message for processing
  this->incomingContacts.push_back(_msg);

  // Prevent the incomingContacts list to grow indefinitely.
  if (this->incomingContacts.size() > 100)
    this->incomingContacts.pop_front();
}
