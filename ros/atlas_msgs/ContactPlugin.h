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
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  /// \brief A plugin for a contact sensor.
  class ContactPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    // Documentation inherited
    protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback that recieves the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Callback for contact messages from the physics engine.
    private: void OnContacts(ConstContactsPtr &_msg);

/*    /// \brief Connect a signal that is triggered when the model is
    /// updated.
    /// \param[in] _subscriber Callback that receives the signal.
    /// \return A pointer to the connection. This must be kept in scope.
    /// \sa Sensor::DisconnectUpdated
    public: template<typename T>
            event::ConnectionPtr ConnectUpdated(T _subscriber)
            {return this->updated.Connect(_subscriber);}

    /// \brief Disconnect from a the updated signal.
    /// \param[in] _c The connection to disconnect
    /// \sa Sensor::ConnectUpdated
    public: void DisconnectUpdated(event::ConnectionPtr &_c)
            {this->updated.Disconnect(_c);}*/

    /// \brief Pointer to the contact sensor
    //private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Transport node used for subscribing to contact sensor messages.
    private: transport::NodePtr node;

    /// \brief Subscription to contact messages from the physics engine
    private: transport::SubscriberPtr contactSub;

    /// \brief Mutex to protect reads and writes.
    private: mutable boost::mutex mutex;

    /// \brief Contacts message used to output sensor data.
    private: msgs::Contacts contactsMsg;

    typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;
    private: ContactMsgs_L incomingContacts;

    /// \brief Collisions this plugin monitors for contacts
    private: std::vector<std::string> collisions;

    /// \brief Pointer to world.
    private: physics::WorldPtr world;

    /// \brief Output contact information.
    private: transport::PublisherPtr contactsPub;

    /// \brief Event triggered when a model is updated.
    //private: event::EventT<void()> updated;

    private: sdf::ElementPtr sdf;

    private: physics::ModelPtr model;
  };
}
#endif
