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
/* Desc: A 4-wheeled vehicle
 * Author: Nate Koenig
 */

#ifndef __GAZEBO_RUBBLE_PLUGIN_HH__
#define __GAZEBO_RUBBLE_PLUGIN_HH__

#include <string>
#include <vector>

#include "gazebo/gazebo.hh"

namespace gazebo
{
  class RubblePlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: RubblePlugin();

    /// \brief Load the plugin
    /// \param _world Pointer to world
    /// \param _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin
    public: virtual void Init();

    private: void MakeBox(const std::string &_name, math::Pose &_pose,
                          math::Vector3 &_size);
    private: void MakeSphere(const std::string &_name, math::Pose &_pose,
                             math::Vector3 &_size);
    private: void MakeCylinder(const std::string &_name, math::Pose &_pose,
                               math::Vector3 &_size);


    private: class Obj
             {
               public: math::Vector3 pos;
               public: math::Vector3 size;
               public: int type;
             };

    private: class CompoundObj
             {
               // center position
               public: math::Vector3 pos;

               // Total size
               public: math::Vector3 size;
               public: std::vector<Obj> objects;
             };

    private: void MakeCompound(const std::string &_name, CompoundObj &_obj);
    private: physics::WorldPtr world;
  };
}
#endif
