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

  /// \brief Set the damping and stiffness of various joints
  private: void SetJointDamping();

  /// \brief Internal helper to reduce code duplication.
  private: bool GetAndPushBackJoint(const std::string& _joint_name,
                                    gazebo::physics::Joint_V& _joints);

  private: gazebo::physics::WorldPtr world;
  private: gazebo::physics::ModelPtr model;
  private: sdf::ElementPtr sdf;
  private: std::string side;
  private: gazebo::physics::Joint_V fingerBaseJoints;
  private: gazebo::physics::Joint_V fingerBaseRotationJoints;
  private: std::vector<gazebo::physics::Joint_V> fingerFlexTwistJoints;

  private: static const int numFingers = 3;
  private: static const int numFlexLinks = 8;

  // Note:
  // erp = dt * kp / ( dt * kp + kd )
  // cfm = 1 / ( dt * kp + kd )
  // or
  // kp = erp / (dt * cfm)
  // kd = (1 - erp) / cfm

  // TODO: make these constants configurable
  private: static const double flexJointCFM = 9.0;
  private: static const double flexJointERP = 0.1;
  private: static const double twistJointCFM = 0.48;
  private: static const double twistJointERP = 0.05;
  private: static const double baseJointCFM = 9.0;
  private: static const double baseJointERP = 0.1;
};

IRobotHandPlugin::IRobotHandPlugin()
{
}

IRobotHandPlugin::~IRobotHandPlugin()
{
}

void IRobotHandPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
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

  this->SetJointDamping();
}

bool IRobotHandPlugin::GetAndPushBackJoint(const std::string& _joint_name,
                                           gazebo::physics::Joint_V& _joints)
{
  gazebo::physics::JointPtr joint = this->model->GetJoint(_joint_name);
  if(!joint)
  {
    gzerr << "Failed to find joint: " << _joint_name << 
      "; aborting plugin load." << std::endl;
    return false;
  }
  _joints.push_back(joint);
  gzlog << "IRobotHandPlugin found joint: " << _joint_name << std::endl;
  return true;
}

bool IRobotHandPlugin::FindJoints()
{
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
      if(!this->GetAndPushBackJoint(joint_name, 
            this->fingerBaseRotationJoints))
        return false;
    }

    // Get the base joint
    snprintf(joint_name, sizeof(joint_name),
      "%s_finger[%d]/joint_base",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerBaseJoints))
      return false;

    // Get the first pair of flex/twist joints
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_flex_from_proximal_to_1",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerFlexTwistJoints[f]))
      return false;
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_twist_from_proximal_to_1",
      this->side.c_str(), f);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerFlexTwistJoints[f]))
      return false;

    // Get the sequence of flex/twist joints, one pair at a time.
    for(int l=1; l<(this->numFlexLinks+1); l++)
    {
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_flex_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      if(!this->GetAndPushBackJoint(joint_name, 
            this->fingerFlexTwistJoints[f]))
        return false;
      snprintf(joint_name, sizeof(joint_name), 
        "%s_finger[%d]/flexible_joint_twist_from_%d_to_%d",
        this->side.c_str(), f, l, l+1);
      if(!this->GetAndPushBackJoint(joint_name, 
            this->fingerFlexTwistJoints[f]))
        return false;
    }

    // Get the last pair of flex/twist joints
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_flex_from_%d_to_distal",
      this->side.c_str(), f, this->numFlexLinks+1);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerFlexTwistJoints[f]))
      return false;
    snprintf(joint_name, sizeof(joint_name), 
      "%s_finger[%d]/flexible_joint_twist_from_%d_to_distal",
      this->side.c_str(), f, this->numFlexLinks+1);
    if(!this->GetAndPushBackJoint(joint_name, 
          this->fingerFlexTwistJoints[f]))
      return false;
  }

  gzlog << "IRobotHandPlugin found all joints for " << this->side
        << " hand." << std::endl;
  return true;
}

void IRobotHandPlugin::SetJointDamping()
{
  // Fake springiness by setting joint limits to 0 and modifying cfm/erp.
  // TODO: implement a generic spring in Gazebo that will work with any
  // physics engine.

  // Handle the flex/twist joints in the flexible section
  for(std::vector<gazebo::physics::Joint_V>::iterator it = 
        this->fingerFlexTwistJoints.begin();
      it != this->fingerFlexTwistJoints.end();
      ++it)
  {
    for(gazebo::physics::Joint_V::iterator iit = it->begin();
        iit != it->end();
        ++iit)
    {
      // Assume that the joints are ordered flex then twist, in pairs.

      (*iit)->SetLowStop(0, 0);
      (*iit)->SetHighStop(0, 0);
      (*iit)->SetAttribute("stop_cfm", 0, this->flexJointCFM);
      (*iit)->SetAttribute("stop_erp", 0, this->flexJointERP);

      ++iit;
      if(iit == it->end())
      {
        gzerr << "Unmatched pair of joints." << std::endl;
        return;
      }
      (*iit)->SetLowStop(0, 0);
      (*iit)->SetHighStop(0, 0);
      (*iit)->SetAttribute("stop_cfm", 0, this->twistJointCFM);
      (*iit)->SetAttribute("stop_erp", 0, this->twistJointERP);
    }
  }

  // Handle the base joints, which are spring-loaded.
  for(gazebo::physics::Joint_V::iterator it = this->fingerBaseJoints.begin();
      it != this->fingerBaseJoints.end();
      ++it)
  {
    (*it)->SetLowStop(0, 0);
    (*it)->SetHighStop(0, 0);
    (*it)->SetAttribute("stop_cfm", 0, this->baseJointCFM);
    (*it)->SetAttribute("stop_erp", 0, this->baseJointERP);
  }
}

GZ_REGISTER_MODEL_PLUGIN(IRobotHandPlugin)
