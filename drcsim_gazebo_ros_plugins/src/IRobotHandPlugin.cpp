/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

class IRobotHandPlugin : public gazebo::ModelPlugin
{
  /// \brief Constructor
  public: IRobotHandPlugin();

  /// \brief Destructor
  public: virtual ~IRobotHandPlugin();

  /// \brief Load the controller
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Grab pointers to all the joints we're going to use.
  /// \return true on success, false otherwise
  private: bool FindJoints();

  /// \brief Set the stiffness of the flexible joints using the provided cfm and
  /// erp values.
  private: void SetFlexDamping(double cfm, double erp);

  private: gazebo::physics::WorldPtr world;
  private: gazebo::physics::ModelPtr model;
  private: sdf::ElementPtr sdf;
  private: std::string side;

  private: static const int numFingers = 3;
  private: static const int numFlexLinks = 8;
  // TODO: make this value configurable
  private: static const double flexJointCFM = 0.0;
  // TODO: make this value configurable
  private: static const double flexJointERP = 0.2;
  private: std::vector<gazebo::physics::Joint_V> fingerBaseJoints;
  private: std::vector<gazebo::physics::Joint_V> fingerBaseRotationJoints;
  private: std::vector<gazebo::physics::Joint_V> fingerFlexTwistJoints;
};

IRobotHandPlugin::IRobotHandPlugin()
{
}

IRobotHandPlugin::~IRobotHandPlugin()
{
}

void IRobotHandPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->sdf = _sdf;

  if(!this->sdf->HasElement("side") || 
     !this->sdf->GetElement("side")->GetValue()->Get(this->side) ||
     ((this->side != "left") && (this->side != "right")))
  {
    gzerr << "Failed to determine which hand we're controlling; "
             "aborting plugin load." << std::endl;
    return;
  }

  gzlog << "IRobotHandPlugin loading for " << this->side << " hand." << 
    std::endl;

  if(!this->FindJoints())
    return;

  this->SetFlexDamping(this->flexJointCFM, this->flexJointERP);
}

bool IRobotHandPlugin::FindJoints()
{
  this->fingerBaseJoints.resize(this->numFingers);
  this->fingerBaseRotationJoints.resize(this->numFingers);
  this->fingerFlexTwistJoints.resize(this->numFingers);

  // Load up the joints we expect to use, finger by finger.
  gazebo::physics::JointPtr joint;
  char joint_name[256];
  for(int f=0; f<this->numFingers; f++)
  {
    // Get the base rotation joint (only fingers 0 and 1)
    if((f==0) || (f==1))
    {
      snprintf(joint_name, sizeof(joint_name),
        "%s_finger[%d]/joint_base_rotation",
        this->side.c_str(), f);
      joint = this->model->GetJoint(joint_name);
      if(!joint)
      {
        gzerr << "Failed to find joint: " << joint_name << 
          "; aborting plugin load." << std::endl;
        return false;
      }
      this->fingerBaseRotationJoints[f].push_back(joint);
    }

    // Get the base joint
    snprintf(joint_name, sizeof(joint_name),
      "%s_finger[%d]/joint_base",
      this->side.c_str(), f);
    joint = this->model->GetJoint(joint_name);
    if(!joint)
    {
      gzerr << "Failed to find joint: " << joint_name << 
        "; aborting plugin load." << std::endl;
      return false;
    }
    this->fingerBaseJoints[f].push_back(joint);

    // Get the flex/twist joints
    for(int l=1; l<(this->numFlexLinks+1); l++)
    {
      // Get the flex joint
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_flex_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      joint = this->model->GetJoint(joint_name);
      if(!joint)
      {
        gzerr << "Failed to find joint: " << joint_name << 
          "; aborting plugin load." << std::endl;
        return false;
      }
      this->fingerFlexTwistJoints[f].push_back(joint);

      // Get the twist joint
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_twist_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      joint = this->model->GetJoint(joint_name);
      if(!joint)
      {
        gzerr << "Failed to find joint: " << joint_name << 
          "; aborting plugin load." << std::endl;
        return false;
      }
      this->fingerFlexTwistJoints[f].push_back(joint);
    }
  }

  gzlog << "IRobotHandPlugin found all joints for " << this->side << " hand." <<
    std::endl;
  return true;
}

void IRobotHandPlugin::SetFlexDamping(double cfm, double erp)
{
  for(std::vector<gazebo::physics::Joint_V>::iterator it = 
        this->fingerFlexTwistJoints.begin();
      it != this->fingerFlexTwistJoints.end();
      ++it)
  {
    for(gazebo::physics::Joint_V::iterator iit = it->begin();
        iit != it->end();
        ++iit)
    {
      // Fake springiness by setting joint limits to 0 and modifying cfm/erp.
      // TODO: implement a generic spring in Gazebo that will work with any
      // physics engine.
      (*iit)->SetLowStop(0, 0);
      (*iit)->SetHighStop(0, 0);
      (*iit)->SetAttribute("cfm", 0, cfm);
      (*iit)->SetAttribute("erp", 0, erp);
    }
  }

}

GZ_REGISTER_MODEL_PLUGIN(IRobotHandPlugin)
