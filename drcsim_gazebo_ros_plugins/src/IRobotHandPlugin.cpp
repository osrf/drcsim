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

#include "drcsim_gazebo_ros_plugins/IRobotHandPlugin.h"

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

  this->SetJointSpringDamper();
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

void IRobotHandPlugin::KpKdToCFMERP(const double _dt,
                           const double _kp, const double _kd,
                           double &_cfm, double &_erp)
{
  /// \TODO: check for NaN cases
  _erp = _dt * _kp / ( _dt * _kp + _kd );
  _cfm = 1.0 / ( _dt * _kp + _kd );
}

void IRobotHandPlugin::CFMERPToKpKd(const double _dt,
                           const double _cfm, const double _erp,
                           double &_kp, double &_kd)
{
  /// \TODO: check for NaN cases
  _kp = _erp / (_dt * _cfm);
  _kd = (1.0 - _erp) / _cfm;
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

void IRobotHandPlugin::SetJointSpringDamper()
{
  // Fake springiness by setting joint limits to 0 and modifying cfm/erp.
  // TODO: implement a generic spring in Gazebo that will work with any
  // physics engine.

  // 10 flex joints @ 0.029 in-lbs/deg per iRobot estimates
  const double flexJointKp = 0.187733 * 10.0;
  const double flexJointKd = 0.1;  // wild guess
  const double twistJointKp = 0.187733 * 10.0 * 2.0;  // wild guess
  const double twistJointKd = 0.1;  // wild guess
  // 0.0031 in-lbs / deg per iRobot estimates
  const double baseJointKp = 0.020068;
  const double baseJointKd = 0.1;  // wild guess

  // 0.23 in-lbs preload per iRobot data
  const double baseJointPreloadTorque = 0.0259865;
  // calculate position for preloaded tension
  const double baseJointPreloadJointPosition =
    baseJointPreloadTorque / baseJointKp;

  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   flexJointKp, flexJointKd, this->flexJointCFM, this->flexJointERP);
  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   twistJointKp, twistJointKd, this->twistJointCFM, this->twistJointERP);
  // this->KpKdToCFMERP(this->world->GetPhysicsEngine()->GetMaxStepSize(),
  //   baseJointKp, baseJointKd, this->baseJointCFM, this->baseJointERP);

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
      (*iit)->SetStiffnessDamping(0, flexJointKp, flexJointKd);
      // (*iit)->SetAttribute("lo_stop", 0, 0.0);
      // (*iit)->SetAttribute("hi_stop", 0, 0.0);
      // (*iit)->SetAttribute("stop_cfm", 0, this->flexJointCFM);
      // (*iit)->SetAttribute("stop_erp", 0, this->flexJointERP);

      ++iit;
      if(iit == it->end())
      {
        gzerr << "Unmatched pair of joints." << std::endl;
        return;
      }
      (*iit)->SetStiffnessDamping(0, twistJointKp, twistJointKd);
      // (*iit)->SetAttribute("lo_stop", 0, 0.0);
      // (*iit)->SetAttribute("hi_stop", 0, 0.0);
      // (*iit)->SetAttribute("stop_cfm", 0, this->twistJointCFM);
      // (*iit)->SetAttribute("stop_erp", 0, this->twistJointERP);
    }
  }

  // Handle the base joints, which are spring-loaded.
  for(gazebo::physics::Joint_V::iterator it = this->fingerBaseJoints.begin();
      it != this->fingerBaseJoints.end();
      ++it)
  {
    (*it)->SetStiffnessDamping(0, baseJointKp, baseJointKd,
      -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("lo_stop", 0, -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("hi_stop", 0, -baseJointPreloadJointPosition);
    // (*it)->SetAttribute("stop_cfm", 0, this->baseJointCFM);
    // (*it)->SetAttribute("stop_erp", 0, this->baseJointERP);
  }
}

GZ_REGISTER_MODEL_PLUGIN(IRobotHandPlugin)
