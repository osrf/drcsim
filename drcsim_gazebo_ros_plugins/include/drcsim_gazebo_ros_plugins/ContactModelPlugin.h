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
#ifndef _GAZEBO_CONTACT_MODEL_PLUGIN_H_
#define _GAZEBO_CONTACT_MODEL_PLUGIN_H_

#include <string>
#include <list>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/unordered/unordered_set.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  /// \brief Contact Model Plugin
  class ContactModelPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ContactModelPlugin();

    /// \brief Destructor.
    public: virtual ~ContactModelPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    // Documentation inherited
    protected: virtual void Load(physics::ModelPtr _model,
        sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback that receives the world's update begin signal.
    private: virtual void OnUpdate();

    /// \brief Callback for contact messages from the physics engine.
    /// \param[in] _msg Gazebo contact message
    private: void OnContacts(ConstContactsPtr &_msg);

    /// \brief Connection that maintains a link between the world's update
    /// begin signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Transport node used for subscribing to contact messages.
    private: transport::NodePtr node;

    /// \brief Subscription to contact messages from the physics engine
    private: transport::SubscriberPtr contactSub;

    /// \brief Mutex to protect reads and writes.
    private: mutable boost::mutex mutex;

    /// \brief Contacts message used to output contact data.
    private: msgs::Contacts contactsMsg;

    typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

    /// \brief A list of incoming contact messages.
    private: ContactMsgs_L incomingContacts;

    /// \brief Collisions this plugin monitors for contacts
    private: boost::unordered_set<std::string> collisions;

    /// \brief Pointer to world.
    private: physics::WorldPtr world;

    /// \brief Output contact information.
    private: transport::PublisherPtr contactsPub;

    /// \brief SDF for this plugin
    private: sdf::ElementPtr sdf;

    /// \brief Model this plugin is attached to.
    private: physics::ModelPtr model;

    /// \brief Custom contact filter created for this plugin.
    private: std::string filterTopicName;
  };
}
#endif
