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

#include "gazebo/math/Rand.hh"
#include "RubblePlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(RubblePlugin)

/////////////////////////////////////////////////
RubblePlugin::RubblePlugin()
{
}

/////////////////////////////////////////////////
void RubblePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  math::Vector3 bottomRight = _sdf->GetValueVector3("bottom_right");
  math::Vector3 topLeft = _sdf->GetValueVector3("top_left");
  math::Vector3 minSize = _sdf->GetValueVector3("min_size");
  math::Vector3 maxSize = _sdf->GetValueVector3("max_size");
  unsigned int count = _sdf->GetValueUInt("count");

  std::vector<CompoundObj> objects;
  std::vector<CompoundObj>::iterator iter;

  for (unsigned int i = 0; i < count; ++i)
  {
    int type = math::Rand::GetIntUniform(0,2);

    Obj obj;

    obj.pos.x = math::Rand::GetDblUniform(bottomRight.x, topLeft.x);
    obj.pos.y = math::Rand::GetDblUniform(bottomRight.y, topLeft.y);
    obj.pos.z = math::Rand::GetDblUniform(bottomRight.z, topLeft.z);

    obj.type = type;

    // Make a box
    if (type == 0)
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.y = math::Rand::GetDblUniform(minSize.y, maxSize.y);
      obj.size.z = math::Rand::GetDblUniform(minSize.z, maxSize.z);
    }
    // Make  a sphere
    else if (type == 1)
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.y = obj.size.z = obj.size.x;
    }
    // Make a cylinder
    else if (type == 2)
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.z = math::Rand::GetDblUniform(minSize.z, maxSize.z);
      obj.size.y = obj.size.x;
    }

    // Make sure the bottom of the rubble piece is about the bottomRight.z
    // This will prevent ground penetration.
    if (obj.pos.z - obj.size.z * 0.5 < bottomRight.z)
      obj.pos.z = bottomRight.z + obj.size.z * 0.5;

    bool merged = false;
    for (iter = objects.begin(); iter != objects.end(); ++iter)
    {
      bool x = fabs(obj.pos.x - (*iter).pos.x) <=
        (*iter).size.x * 0.5 + obj.size.x * 0.5;
      bool y = fabs(obj.pos.y - (*iter).pos.y) <=
        (*iter).size.y * 0.5 + obj.size.y * 0.5;
      bool z = fabs(obj.pos.z - (*iter).pos.z) <=
        (*iter).size.z * 0.5 + obj.size.z * 0.5;

      if (x && y && z)
      {
        (*iter).objects.push_back(obj);
        std::vector<Obj>::iterator objIter;

        (*iter).pos.Set(0, 0, 0);
        math::Vector3 min, max;
        min.x = min.y = min.z = std::numeric_limits<double>::max();
        max.x = max.y = max.z = std::numeric_limits<double>::min();

        for (objIter = (*iter).objects.begin();
             objIter != (*iter).objects.end(); ++objIter)
        {
          (*iter).pos += (*objIter).pos;
          min.x = std::min(min.x, (*objIter).pos.x - (*objIter).size.x * 0.5);
          min.y = std::min(min.y, (*objIter).pos.y - (*objIter).size.y * 0.5);
          min.z = std::min(min.z, (*objIter).pos.z - (*objIter).size.z * 0.5);

          max.x = std::max(max.x, (*objIter).pos.x + (*objIter).size.x * 0.5);
          max.y = std::max(max.y, (*objIter).pos.y + (*objIter).size.y * 0.5);
          max.z = std::max(max.z, (*objIter).pos.z + (*objIter).size.z * 0.5);
        }

        // Recalculate the middle position of the compound object
        (*iter).pos /= (*iter).objects.size();

        // Recalculate the total size of the compound object
        (*iter).size.Set(max.x - min.x, max.y - min.y, max.z - min.z);

        merged = true;
        break;
      }
    }

    if (!merged)
    {
      RubblePlugin::CompoundObj co;
      co.pos = obj.pos;
      co.size = obj.size;
      co.objects.push_back(obj);
      objects.push_back(co);
    }
  }

  int i =0; 
  for (iter = objects.begin(); iter != objects.end(); ++iter, ++i)
  {
    std::ostringstream name;
    name << "rubble_" << i;
    this->MakeCompound(name.str(), *iter);
  }

  /*if (type == 0)
    this->MakeBox(name.str(), pose, size);
  else if (type == 1)
    this->MakeSphere(name.str(), pose, size);
  else
    this->MakeCylinder(name.str(), pose, size);
    */

}

/////////////////////////////////////////////////
void RubblePlugin::Init()
{
}

void RubblePlugin::MakeSphere(const std::string &_name, math::Pose &_pose,
                              math::Vector3 &_size)
{
  std::ostringstream newModelStr;

  newModelStr << "<gazebo version ='1.0'>\
    <model name='" << _name << "'>\
    <origin pose='" << _pose << "'/>\
    <link name ='link'>\
      <collision name ='collision'>\
        <mass>0.5</mass>\
        <geometry>\
          <sphere radius='" << _size.x * 0.5 << "'/>\
        </geometry>\
      </collision>\
      <visual name ='visual'>\
        <geometry>\
          <sphere radius='" << _size.x * 0.5 << "'/>\
        </geometry>\
        <material script ='Gazebo/Grey'/>\
      </visual>\
    </link>\
  </model>\
  </gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

void RubblePlugin::MakeBox(const std::string &_name, math::Pose &_pose,
                           math::Vector3 &_size)
{
  std::ostringstream newModelStr;

  newModelStr << "<gazebo version ='1.0'>\
    <model name='" << _name << "'>\
    <origin pose='" << _pose << "'/>\
    <link name ='link'>\
      <collision name ='collision'>\
        <mass>0.5</mass>\
        <geometry>\
          <box size ='" << _size << "'/>\
        </geometry>\
      </collision>\
      <visual name ='visual'>\
        <geometry>\
          <box size ='" << _size << "'/>\
        </geometry>\
        <material script ='Gazebo/Grey'/>\
      </visual>\
    </link>\
  </model>\
  </gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

void RubblePlugin::MakeCylinder(const std::string &_name, math::Pose &_pose,
                                math::Vector3 &_size)
{
  std::ostringstream newModelStr;

  newModelStr << "<gazebo version ='1.0'>\
    <model name='" << _name << "'>\
    <origin pose='" << _pose << "'/>\
    <link name ='link'>\
      <collision name ='collision'>\
        <mass>0.5</mass>\
        <geometry>\
          <cylinder radius='" << _size.x * 0.5 
                              << "' length='" << _size.z << "'/>\
        </geometry>\
      </collision>\
      <visual name ='visual'>\
        <geometry>\
          <cylinder radius='" << _size.x * 0.5 
                              << "' length='" << _size.z << "'/>\
        </geometry>\
        <material script ='Gazebo/Grey'/>\
      </visual>\
    </link>\
  </model>\
  </gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

void RubblePlugin::MakeCompound(const std::string &_name, CompoundObj &_obj)
{
  std::ostringstream newModelStr, geomStr;

  newModelStr << "<gazebo version ='1.0'>"
      << "<model name='" << _name << "'>"
      << "  <origin pose='" << _obj.pos << " 0 0 0'/>"
      << "  <link name ='link'>";

  int i = 0;
  for (std::vector<Obj>::iterator iter = _obj.objects.begin();
       iter != _obj.objects.end(); ++iter, ++i)
  {
    if ((*iter).type == 0)
      geomStr << "<box size='" << (*iter).size << "'/>";
    else if ((*iter).type == 1)
      geomStr << "<sphere radius='" << (*iter).size.x * 0.5 << "'/>";
    else
      geomStr << "<cylinder radius='" << (*iter).size.x * 0.5 
                  << "' length='" << (*iter).size.z << "'/>";

    newModelStr << "    <collision name ='collision_" << i << "'>"
                << "      <origin pose='" << (*iter).pos << " 0 0 0'/>"
                << "      <mass>0.2</mass>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "    </collision>"
                << "    <visual name ='visual_" << i << "'>"
                << "      <origin pose='" << (*iter).pos << " 0 0 0'/>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "      <material script ='Gazebo/Grey'/>"
                << "    </visual>";
  }

  newModelStr << "  </link>"
              << "</model>"
              << "</gazebo>";

  this->world->InsertModelString(newModelStr.str());
}
