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

#include <string>
#include <algorithm>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "AtlasPlugin.h"

#include "sensor_msgs/Imu.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

////////////////////////////////////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
  // the parent link of the imu_sensor ends up being pelvis after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_senosr block.
  this->imuLinkName = "pelvis";

  this->pelvisLinkName = "pelvis";

  // initialize behavior library
  this->atlasSimInterface = create_atlas_sim_interface();
  this->usingWalkingController = false;
}

////////////////////////////////////////////////////////////////////////////////
AtlasPlugin::~AtlasPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
  // shutdown behavior library
  destroy_atlas_sim_interface();
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();

  // JoitnController: built-in gazebo to control joints
  this->jointController = this->model->GetJointController();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->jointCmdPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + this->model->GetName() + "/joint_cmd");

  // save sdf
  this->sdf = _sdf;

  /// \TODO: get this from sdf
  this->timeConstant = 0.005;

  // initialize update time
  this->lastControllerUpdateTime = this->world->GetSimTime();
  this->lastFTUpdateTime = this->world->GetSimTime();

  // initialize imu
  this->lastImuTime = this->world->GetSimTime();

  // get joints
  this->jointNames.push_back("back_lbz");
  this->jointNames.push_back("back_mby");
  this->jointNames.push_back("back_ubx");
  this->jointNames.push_back("neck_ay");
  this->jointNames.push_back("l_leg_uhz");
  this->jointNames.push_back("l_leg_mhx");
  this->jointNames.push_back("l_leg_lhy");
  this->jointNames.push_back("l_leg_kny");
  this->jointNames.push_back("l_leg_uay");
  this->jointNames.push_back("l_leg_lax");
  this->jointNames.push_back("r_leg_uhz");
  this->jointNames.push_back("r_leg_mhx");
  this->jointNames.push_back("r_leg_lhy");
  this->jointNames.push_back("r_leg_kny");
  this->jointNames.push_back("r_leg_uay");
  this->jointNames.push_back("r_leg_lax");
  this->jointNames.push_back("l_arm_usy");
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back("l_arm_uwy");
  this->jointNames.push_back("l_arm_mwx");
  this->jointNames.push_back("r_arm_usy");
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back("r_arm_uwy");
  this->jointNames.push_back("r_arm_mwx");

  this->joints.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])
    {
      ROS_ERROR("atlas robot expected joint[%s] not present, plugin not loaded",
        this->jointNames[i].c_str());
      return;
    }
  }

  {
    // do some analysis
    physics::LinkPtr utorso = this->model->GetLink("utorso");
    physics::LinkPtr mtorso = this->model->GetLink("mtorso");
    physics::LinkPtr ltorso = this->model->GetLink("ltorso");
    physics::LinkPtr pelvis = this->model->GetLink("pelvis");

    physics::Inertial i_utorso = *utorso->GetInertial();
    physics::Inertial i_mtorso = *mtorso->GetInertial();
    physics::Inertial i_ltorso = *ltorso->GetInertial();
    physics::Inertial i_pelvis = *pelvis->GetInertial();

    physics::LinkPtr l_lleg  = this->model->GetLink("l_lleg");
    physics::LinkPtr l_talus = this->model->GetLink("l_talus");
    physics::LinkPtr l_foot  = this->model->GetLink("l_foot");
    physics::LinkPtr r_lleg  = this->model->GetLink("r_lleg");
    physics::LinkPtr r_talus = this->model->GetLink("r_talus");
    physics::LinkPtr r_foot  = this->model->GetLink("r_foot");

    physics::Inertial i_r_lleg  = *r_lleg->GetInertial();
    physics::Inertial i_r_talus = *r_talus->GetInertial();
    physics::Inertial i_r_foot  = *r_foot->GetInertial();

    physics::LinkPtr r_uglut = this->model->GetLink("r_uglut");
    physics::LinkPtr r_lglut = this->model->GetLink("r_lglut");
    physics::LinkPtr r_uleg = this->model->GetLink("r_uleg");

    physics::Inertial i_r_uglut = *r_uglut->GetInertial();
    physics::Inertial i_r_lglut = *r_lglut->GetInertial();
    physics::Inertial i_r_uleg  = *r_uleg->GetInertial();

    std::cout << "inertia of utorso:\n" << i_utorso << "\n";
    std::cout << "inertia of mtorso:\n" << i_mtorso << "\n";
    std::cout << "inertia of ltorso:\n" << i_ltorso << "\n";
    std::cout << "inertia of pelvis:\n" << i_pelvis << "\n";

    std::cout << "inertia of lleg:\n" << i_r_lleg << "\n";
    std::cout << "inertia of talus:\n" << i_r_talus << "\n";
    std::cout << "inertia of foot:\n" << i_r_foot << "\n";

    std::cout << "inertia of uglut:\n" << i_r_uglut << "\n";
    std::cout << "inertia of lglut:\n" << i_r_lglut << "\n";
    std::cout << "inertia of uleg:\n" << i_r_uleg << "\n";

    // transform from r_foot link frame to r_lleg link
    math::Pose r_foot_talus =
      r_talus->GetWorldPose() - r_foot->GetWorldPose();

    // transform from r_talus link frame to r_lleg link
    math::Pose r_talus_lleg =
      r_lleg->GetWorldPose() - r_talus->GetWorldPose();

    // transform from r_foot link frame to r_lleg link
    math::Pose r_foot_lleg =
      r_lleg->GetWorldPose() - r_foot->GetWorldPose();

    // total inertia from knee down viewed in the lleg link frame
    physics::Inertial i_r_lleg_talus_foot =
      i_r_lleg.GetInertial(math::Pose()) +
      i_r_talus.GetInertial(r_talus_lleg) +
      i_r_foot.GetInertial(r_foot_lleg);
    std::cout << "TOTAL inertia of foot, talus and lleg at lleg link:\n"
          << i_r_lleg_talus_foot << "\n";

    // total inertia from talus down viewed in the talus link frame
    physics::Inertial i_r_talus_foot =
      i_r_talus.GetInertial(math::Pose()) +
      i_r_foot.GetInertial(r_foot_talus);
    std::cout << "TOTAL inertia of foot and talus at talus link:\n"
          << i_r_talus_foot << "\n";

    ////////////////////////////////////////////////////////////////////////////
    // Tweak Inertias
    //
    // Strategy for tweaking ankle inertia:
    //  - keep max inertia ratio or mass ratio less than 10 to 1
    //  - keep combined mass equal
    //  - keep combined MOI(talus + foot) about uay constant
    //  - (try) keep combined MOI(talus + foot + lleg) about kny constant
    //
    // Strategy for tweaking spine inertia:
    //  - tweak as two separate systems
    //   * utorso(+60lbs for backpack) + mtorso
    //   * ltorso + pelvis
    // For each spine subsystem:
    //  - keep max inertia ratio or mass ratio less than 10 to 1
    //  - keep combined mass equal
    //  - keep combined MOI(utorso + mtorso) about mby joint constant
    //  - keep combined MOI(ltorso + pelvis) about mby constant
    //
    // Side effects:
    //  - CG of subsystems shifts
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    //  From a controls perspective, important things to keep constant
    //  Feet
    //   1a distribute foot mass and talus mass equally
    //      check how much cog of system has moved.
    //   1b keep IYY of talus + foot about uay constant by tweaking foot IYY
    //      *keep IYY of lleg + talus + foot about kny constant
    //      decrease foot IYY and increase talus IYY simultaneously
    //   1c keep IXX of foot about lax(uay) constant
    //      increase talus IXX to reduce IXX ratio between talus and foot
    //      check overall IXX increase of talus, foot and lleg, check % is small
    //   1d keep IZZ of talus + foot about uay(lax) constant (avg is sufficient)
    //      split IZZ equally between talus and uay

    std::cout << "================= Ankle Inertial Tweak ==================\n";

    physics::Inertial i_r_talus_mod = i_r_talus;
    physics::Inertial i_r_foot_mod = i_r_foot;

    // 1a modify mass by average
    i_r_talus_mod.SetMass(0.5 * (i_r_talus.GetMass() + i_r_foot.GetMass()));
    i_r_foot_mod.SetMass(0.5 * (i_r_talus.GetMass() + i_r_foot.GetMass()));
    // 1a check cog movement is small
    {
      physics::Inertial i1 = i_r_talus + i_r_foot;
      physics::Inertial i1_mod = i_r_talus_mod + i_r_foot_mod;
      std::cout << "\ncheck cog movement : \n"
            << "  original cog: [" << i1.GetPose() << "]\n"
            << "  modified cog: [" << i1_mod.GetPose() << "]\n"
            << "  changed  cog: [" << i1_mod.GetPose() - i1.GetPose() << "]\n";
    }

    // 1b modify IYY, check total MOI at uay
    double iyy_talus = i_r_talus_mod.GetIYY();
    double iyy_foot = i_r_foot_mod.GetIYY();
    i_r_talus_mod.SetIYY(0.5*(iyy_talus + iyy_foot));
    i_r_foot_mod.SetIYY(0.5*(iyy_talus + iyy_foot));
    // check total IYY about uay
    math::Matrix3 moi_talus_foot_uay =
            (i_r_talus.GetMOI(math::Pose()) +
             i_r_foot.GetMOI(r_foot_talus));
    math::Matrix3 moi_talus_foot_uay_mod =
            (i_r_talus_mod.GetMOI(math::Pose()) +
             i_r_foot_mod.GetMOI(r_foot_talus));
    std::cout << "\nCheck IYY of talus + foot about uay:\n"
          << "  original iyy: " << moi_talus_foot_uay[2][2] << "\n"
          << "  modified iyy: " << moi_talus_foot_uay_mod[2][2] << "\n"
          << "  changed  iyy: " << moi_talus_foot_uay_mod[2][2] -
                                   moi_talus_foot_uay[2][2] << "\n";

    // more iterations to make changed iyy 0

    // 1c modify IXX, simply increase talus IXX and check % increase for
    //   lleg + talus + foot about kny
    double ixx_talus = i_r_talus_mod.GetIXX();
    double ixx_foot = i_r_foot_mod.GetIXX();
    // simply brute force ixx of talus to be 1/2 of ixx foot
    i_r_talus_mod.SetIXX(0.0*ixx_talus + 0.5*ixx_foot);
    // check increase of IXX for lleg + talus + foot about lleg (kny location)
    physics::Inertial i_r_lleg_talus_foot_mod =
      i_r_lleg.GetInertial(math::Pose()) +
      i_r_talus_mod.GetInertial(r_talus_lleg) +
      i_r_foot_mod.GetInertial(r_foot_lleg);
    std::cout << "\nCheck IXX of foot, talus and lleg at lleg(uay) link:\n"
          << "  original ixx: " << i_r_lleg_talus_foot.GetIXX() << "\n"
          << "  modified ixx: " << i_r_lleg_talus_foot_mod.GetIXX() << "\n"
          << "  changed  ixx: " << i_r_lleg_talus_foot_mod.GetIXX() -
                                   i_r_lleg_talus_foot.GetIXX() << "\n"
          << "  changed% ixx: " << (i_r_lleg_talus_foot_mod.GetIXX() -
                                    i_r_lleg_talus_foot.GetIXX()) /
                                    i_r_lleg_talus_foot.GetIXX() << "\n";

    // 1d modify IZZ by splitting equally
    //    this actually does not affect dynamics, because lleg, talus, foot
    //    are all constrained in z-axis.
    // We have to take more advantage of this property.
    double izz_talus = i_r_talus_mod.GetIZZ();
    double izz_foot = i_r_foot_mod.GetIZZ();
    i_r_talus_mod.SetIZZ(0.5*(izz_talus + izz_foot));
    i_r_foot_mod.SetIZZ(0.5*(izz_talus + izz_foot));
    // check increase of IZZ for lleg + talus + foot about lleg (kny location)
    i_r_lleg_talus_foot_mod =
      i_r_lleg.GetInertial(math::Pose()) +
      i_r_talus_mod.GetInertial(r_talus_lleg) +
      i_r_foot_mod.GetInertial(r_foot_lleg);
    std::cout << "\nCheck IZZ of foot, talus and lleg at lleg(uay) link:\n"
          << "  original izz: " << i_r_lleg_talus_foot.GetIZZ() << "\n"
          << "  modified izz: " << i_r_lleg_talus_foot_mod.GetIZZ() << "\n"
          << "  changed  izz: " << i_r_lleg_talus_foot_mod.GetIZZ() -
                                   i_r_lleg_talus_foot.GetIZZ() << "\n"
          << "  changed% izz: " << (i_r_lleg_talus_foot_mod.GetIZZ() -
                                    i_r_lleg_talus_foot.GetIZZ()) /
                                    i_r_lleg_talus_foot.GetIZZ() << "\n";


    // what's the new inertial
    std::cout << "\n============ Ankle Summary ==============\n";
    std::cout << "original talus mass: \n" << i_r_talus.GetMass() << "\n";
    std::cout << "original talus MOI@cog: \n" << i_r_talus.GetMOI() << "\n";
    std::cout << "modified talus mass: \n" << i_r_talus_mod.GetMass() << "\n";
    std::cout << "modified talus MOI@cog: \n" << i_r_talus_mod.GetMOI() << "\n";
    std::cout << "original foot mass: \n" << i_r_foot.GetMass() << "\n";
    std::cout << "original foot MOI@cog: \n" << i_r_foot.GetMOI() << "\n";
    std::cout << "modified foot mass: \n" << i_r_foot_mod.GetMass() << "\n";
    std::cout << "modified foot MOI@cog: \n" << i_r_foot_mod.GetMOI() << "\n";

    ////////////////////////////////////////////////////////////////////////////
    //  Spine
    //   1a Split utorso and mtorso mass 10:1 ratio
    //   1b keep IYY of mtorso + utorso about mby constant
    //   1c keep IXX of utorso about ubx constant
    //   1d (optional) keep IYY of ltorso + pelvis about mby constant
    //   1e keep IZZ of utorso + mtorso + ltorso about lbz constant
    //      check group CG to make sure changes are reasonable

    std::cout << "\n============ Torso Inertial Tweaks ==============\n";

    physics::Inertial i_utorso_mod = i_utorso;
    physics::Inertial i_mtorso_mod = i_mtorso;
    physics::Inertial i_ltorso_mod = i_ltorso;
    physics::Inertial i_pelvis_mod = i_pelvis;

    // 1a split utorso and mtorso mass by a ratio of about 5:1
    //   anticipate utorso increase by 30kg, so end ratio after
    //   torso change will be around 12:1, which is still tolerable
    double mass_um = i_utorso.GetMass() + i_mtorso.GetMass();
    double e_um = 0.2;
    i_utorso_mod.SetMass((1.0 - e_um) * mass_um);
    i_mtorso_mod.SetMass(        e_um * mass_um);
    // 1a check cog movement is small
    {
      physics::Inertial i1 = i_utorso + i_mtorso;
      physics::Inertial i1_mod = i_utorso_mod + i_mtorso_mod;
      std::cout << "\ncheck cog of utorso + mtorso movement : \n"
            << "  original cog: [" << i1.GetPose() << "]\n"
            << "  modified cog: [" << i1_mod.GetPose() << "]\n"
            << "  changed  cog: [" << i1_mod.GetPose() - i1.GetPose() << "]\n";
    }

    // 1b modify IYY, check total MOI at mby
    //    given utorso and mtorso are constrained about y axis,
    //    it is ok to simply distribute IYY equally.
    double iyy_utorso = i_utorso_mod.GetIYY();
    double iyy_mtorso = i_mtorso_mod.GetIYY();
    i_utorso_mod.SetIYY(0.5*(iyy_utorso + iyy_mtorso));
    i_mtorso_mod.SetIYY(0.5*(iyy_utorso + iyy_mtorso));
    // check total IYY about mby
    // transform from foot link frame to lleg link
    math::Pose mtorso_utorso =
      utorso->GetWorldPose() - mtorso->GetWorldPose();
    math::Matrix3 moi_utorso_mtorso_mby =
            (i_utorso.GetMOI(math::Pose()) +
             i_mtorso.GetMOI(mtorso_utorso));
    math::Matrix3 moi_utorso_mtorso_mby_mod =
            (i_utorso_mod.GetMOI(math::Pose()) +
             i_mtorso_mod.GetMOI(mtorso_utorso));
    std::cout << "\nCheck IYY of utorso + mtorso about mby:\n"
          << "  original iyy: " << moi_utorso_mtorso_mby[2][2] << "\n"
          << "  modified iyy: " << moi_utorso_mtorso_mby_mod[2][2] << "\n"
          << "  changed  iyy: " << moi_utorso_mtorso_mby_mod[2][2] -
                                   moi_utorso_mtorso_mby[2][2] << "\n";

    // 1b, keep IXX of utorso and about ubx constant.
    //     Increase IXX of mtorso, and check to see that change in IXX
    //     of utorso + mtorso about mby is reasonable
    double ixx_mtorso = i_mtorso_mod.GetIXX();
    double ixx_utorso = i_utorso_mod.GetIXX();
    // simply brute force ixx of mtorso to be 1/2 of ixx utorso
    i_mtorso_mod.SetIXX(0.0*ixx_mtorso + 0.5*ixx_utorso);
    // check increase of IXX for mtorso + utorso about mtorso (ubx location)
    physics::Inertial i_mtorso_utorso =
      i_mtorso.GetInertial(math::Pose()) +
      i_utorso.GetInertial(mtorso_utorso);
    physics::Inertial i_mtorso_utorso_mod =
      i_mtorso_mod.GetInertial(math::Pose()) +
      i_utorso_mod.GetInertial(mtorso_utorso);
    std::cout << "\nCheck IXX of utorso, mtorso at mtorso(ubx) link:\n"
          << "  original ixx: " << i_mtorso_utorso.GetIXX() << "\n"
          << "  modified ixx: " << i_mtorso_utorso_mod.GetIXX() << "\n"
          << "  changed  ixx: " << i_mtorso_utorso_mod.GetIXX() -
                                   i_mtorso_utorso.GetIXX() << "\n"
          << "  changed% ixx: " << (i_mtorso_utorso_mod.GetIXX() -
                                    i_mtorso_utorso.GetIXX()) /
                                    i_mtorso_utorso.GetIXX() << "\n";

    // 1c modify IZZ by splitting equally between utorso, mtorso, ltorso
    //    this actually does not affect dynamics, because all three lins
    //    are fully constrained in z-axis.
    // We have to take more advantage of this types of inertia distribution.
    double izz_utorso = i_utorso_mod.GetIZZ();
    double izz_mtorso = i_mtorso_mod.GetIZZ();
    double izz_ltorso = i_ltorso_mod.GetIZZ();
    i_utorso_mod.SetIZZ((izz_utorso + izz_mtorso + izz_ltorso) / 3.0);
    i_mtorso_mod.SetIZZ((izz_utorso + izz_mtorso + izz_ltorso) / 3.0);
    i_ltorso_mod.SetIZZ((izz_utorso + izz_mtorso + izz_ltorso) / 3.0);
    // check increase of IZZ of utorso, mtorso, ltorso about ltorso(lbz)
    // make sure it's reasonable
    math::Pose mtorso_ltorso =
      ltorso->GetWorldPose() - mtorso->GetWorldPose();
    math::Pose utorso_ltorso =
      ltorso->GetWorldPose() - utorso->GetWorldPose();
    physics::Inertial i_utorso_mtorso_ltorso =
      i_utorso.GetInertial(utorso_ltorso) +
      i_mtorso.GetInertial(mtorso_ltorso) +
      i_ltorso.GetInertial(math::Pose());
    physics::Inertial i_utorso_mtorso_ltorso_mod =
      i_utorso_mod.GetInertial(utorso_ltorso) +
      i_mtorso_mod.GetInertial(mtorso_ltorso) +
      i_ltorso_mod.GetInertial(math::Pose());
    std::cout << "\nCheck IZZ of utorso, mtorso and ltoros at ltorso(lbz):\n"
          << "  original izz: " << i_utorso_mtorso_ltorso.GetIZZ() << "\n"
          << "  modified izz: " << i_utorso_mtorso_ltorso_mod.GetIZZ() << "\n"
          << "  changed  izz: " << i_utorso_mtorso_ltorso_mod.GetIZZ() -
                                   i_utorso_mtorso_ltorso.GetIZZ() << "\n"
          << "  changed% izz: " << (i_utorso_mtorso_ltorso_mod.GetIZZ() -
                                    i_utorso_mtorso_ltorso.GetIZZ()) /
                                    i_utorso_mtorso_ltorso.GetIZZ() << "\n";

    // Now, to the same thing
    // Simply increase IYY of ltorso so pelvis:ltorso ratio is 10:1
    // Simply split IXX between ltorso an dpelvis

    // 1a split pelvis and ltorso mass by a ratio of about 10:1
    //   anticipate pelvis increase by 30kg, so end ratio after
    //   torso change will be around 12:1, which is still tolerable
    double mass_pl = i_pelvis.GetMass() + i_ltorso.GetMass();
    double e_pl = 0.1;
    i_pelvis_mod.SetMass((1.0 - e_pl) * mass_pl);
    i_ltorso_mod.SetMass(        e_pl * mass_pl);
    // 1a check cog movement is small
    {
      physics::Inertial i1 = i_pelvis + i_ltorso;
      physics::Inertial i1_mod = i_pelvis_mod + i_ltorso_mod;
      std::cout << "\ncheck cog of pelvis + ltorso movement : \n"
            << "  original cog: [" << i1.GetPose() << "]\n"
            << "  modified cog: [" << i1_mod.GetPose() << "]\n"
            << "  changed  cog: [" << i1_mod.GetPose() - i1.GetPose() << "]\n";
    }

    // 1b modify IYY, check total MOI at mby
    //    given pelvis and ltorso are constrained about y axis,
    //    it is ok to simply distribute IYY equally.
    double iyy_pelvis = i_pelvis_mod.GetIYY();
    double iyy_ltorso = i_ltorso_mod.GetIYY();
    i_pelvis_mod.SetIYY(0.5*(iyy_pelvis + iyy_ltorso));
    i_ltorso_mod.SetIYY(0.5*(iyy_pelvis + iyy_ltorso));
    // check total IYY about mby
    // transform from foot link frame to lleg link
    math::Pose ltorso_pelvis =
      pelvis->GetWorldPose() - ltorso->GetWorldPose();
    math::Matrix3 moi_pelvis_ltorso_mby =
            (i_pelvis.GetMOI(math::Pose()) +
             i_ltorso.GetMOI(ltorso_pelvis));
    math::Matrix3 moi_pelvis_ltorso_mby_mod =
            (i_pelvis_mod.GetMOI(math::Pose()) +
             i_ltorso_mod.GetMOI(ltorso_pelvis));
    std::cout << "\nCheck IYY of pelvis + ltorso about mby:\n"
          << "  original iyy: " << moi_pelvis_ltorso_mby[2][2] << "\n"
          << "  modified iyy: " << moi_pelvis_ltorso_mby_mod[2][2] << "\n"
          << "  changed  iyy: " << moi_pelvis_ltorso_mby_mod[2][2] -
                                   moi_pelvis_ltorso_mby[2][2] << "\n";

    // 1b modify IXX, check total MOI at lbz
    //    given pelvis and ltorso are constrained about x axis,
    //    it is ok to simply distribute IXX equally.
    double ixx_pelvis = i_pelvis_mod.GetIXX();
    double ixx_ltorso = i_ltorso_mod.GetIXX();
    i_pelvis_mod.SetIXX(0.5*(ixx_pelvis + ixx_ltorso));
    i_ltorso_mod.SetIXX(0.5*(ixx_pelvis + ixx_ltorso));
    // check total IXX about lbz
    // transform from foot link frame to lleg link
    math::Matrix3 moi_pelvis_ltorso_lbz =
            (i_pelvis.GetMOI(math::Pose()) +
             i_ltorso.GetMOI(ltorso_pelvis));
    math::Matrix3 moi_pelvis_ltorso_lbz_mod =
            (i_pelvis_mod.GetMOI(math::Pose()) +
             i_ltorso_mod.GetMOI(ltorso_pelvis));
    std::cout << "\nCheck IXX of pelvis + ltorso about lbz:\n"
          << "  original ixx: " << moi_pelvis_ltorso_lbz[2][2] << "\n"
          << "  modified ixx: " << moi_pelvis_ltorso_lbz_mod[2][2] << "\n"
          << "  changed  ixx: " << moi_pelvis_ltorso_lbz_mod[2][2] -
                                   moi_pelvis_ltorso_lbz[2][2] << "\n";



    // what's the new inertial
    std::cout << "\n============ Torso Summary ==============\n";
    std::cout << "original utorso Mass@cog\n" << i_utorso.GetMass() << "\n";
    std::cout << "original utorso MOI@cog\n" << i_utorso.GetMOI() << "\n";
    std::cout << "modified utorso Mass@cog\n" << i_utorso_mod.GetMass() << "\n";
    std::cout << "modified utorso MOI@cog\n" << i_utorso_mod.GetMOI() << "\n";
    std::cout << "original mtorso Mass@cog\n" << i_mtorso.GetMass() << "\n";
    std::cout << "original mtorso MOI@cog\n" << i_mtorso.GetMOI() << "\n";
    std::cout << "modified mtorso Mass@cog\n" << i_mtorso_mod.GetMass() << "\n";
    std::cout << "modified mtorso MOI@cog\n" << i_mtorso_mod.GetMOI() << "\n";
    std::cout << "original ltorso Mass@cog\n" << i_ltorso.GetMass() << "\n";
    std::cout << "original ltorso MOI@cog\n" << i_ltorso.GetMOI() << "\n";
    std::cout << "modified ltorso Mass@cog\n" << i_ltorso_mod.GetMass() << "\n";
    std::cout << "modified ltorso MOI@cog\n" << i_ltorso_mod.GetMOI() << "\n";
    std::cout << "original pelvis Mass@cog\n" << i_pelvis.GetMass() << "\n";
    std::cout << "original pelvis MOI@cog\n" << i_pelvis.GetMOI() << "\n";
    std::cout << "modified pelvis Mass@cog\n" << i_pelvis_mod.GetMass() << "\n";
    std::cout << "modified pelvis MOI@cog\n" << i_pelvis_mod.GetMOI() << "\n";

    
    ////////////////////////////////////////////////////////////////////////////
    //  Gluts
    //   pelvis <uhz>@uglut --> 0.089 m down
    //   uglut <mhx>@lglut --> collocated
    //   lglut <lhy>@uleg --> 0.05, 0, -0.05
    //
    //   note i_pelvis is already modified, continue to modify it.
    //
    //   Split pelvis and uglut mass 10:1 ratio
    //   Split uleg and lglut mass 10:1 ratio
    //   Distribute IZZ across uglut, lglut, uleg
    //   Distribute IXX across pelvis and uglut
    //   Distribute IXX across lglut and uleg
    //   Distribute IYY across pelvis, uglut, lglut
    //
    std::cout << "\n============ Gluts Inertial Tweaks ==============\n";

    physics::Inertial i_pelvis_mod2 = i_pelvis_mod;
    physics::Inertial i_r_uglut_mod = i_r_uglut;
    physics::Inertial i_r_lglut_mod = i_r_lglut;
    physics::Inertial i_r_uleg_mod = i_r_uleg;

    // split pelvis and r_uglut mass by a ratio of about 5:1
    double mass_pu = i_pelvis_mod.GetMass() + i_r_uglut.GetMass();
    double e_pu = 0.1;
    i_pelvis_mod2.SetMass((1.0 - e_pu) * mass_pu);
    i_r_uglut_mod.SetMass(        e_pu * mass_pu);
    // check cog movement is small
    {
      physics::Inertial i1 = i_pelvis_mod + i_r_uglut;
      physics::Inertial i1_mod = i_pelvis_mod2 + i_r_uglut_mod;
      std::cout << "\ncheck cog of pelvis + r_uglut movement : \n"
            << "  original cog: [" << i1.GetPose() << "]\n"
            << "  modified cog: [" << i1_mod.GetPose() << "]\n"
            << "  changed  cog: [" << i1_mod.GetPose() - i1.GetPose() << "]\n";
    }

    // split uleg and r_lglut mass by a ratio of about 5:1
    double mass_ul = i_r_uleg.GetMass() + i_r_lglut.GetMass();
    double e_ul = 0.1;
    i_r_uleg_mod.SetMass((1.0 - e_ul) * mass_ul);
    i_r_lglut_mod.SetMass(        e_ul * mass_ul);
    // check cog movement is small
    {
      physics::Inertial i1 = i_r_uleg + i_r_lglut;
      physics::Inertial i1_mod = i_r_uleg_mod + i_r_lglut_mod;
      std::cout << "\ncheck cog of r_uleg + r_lglut movement : \n"
            << "  original cog: [" << i1.GetPose() << "]\n"
            << "  modified cog: [" << i1_mod.GetPose() << "]\n"
            << "  changed  cog: [" << i1_mod.GetPose() - i1.GetPose() << "]\n";
    }

    // 1c modify IZZ by splitting equally between r_uglut, r_lglut, r_uleg
    //    this actually does not affect dynamics, because all three lins
    //    are fully constrained in z-axis.
    // We have to take more advantage of this types of inertia distribution.
    double izz_r_uglut = i_r_uglut_mod.GetIZZ();
    double izz_r_lglut = i_r_lglut_mod.GetIZZ();
    double izz_r_uleg = i_r_uleg_mod.GetIZZ();
    i_r_uglut_mod.SetIZZ((izz_r_uglut + izz_r_lglut + izz_r_uleg) / 3.0);
    i_r_lglut_mod.SetIZZ((izz_r_uglut + izz_r_lglut + izz_r_uleg) / 3.0);
    i_r_uleg_mod.SetIZZ((izz_r_uglut + izz_r_lglut + izz_r_uleg) / 3.0);
    // check increase of IZZ of r_uglut, r_lglut, r_uleg about r_uleg(lbz)
    // make sure it's reasonable
    math::Pose r_lglut_r_uglut =
      r_uglut->GetWorldPose() - r_lglut->GetWorldPose();
    math::Pose r_uleg_r_uglut =
      r_uglut->GetWorldPose() - r_uleg->GetWorldPose();
    physics::Inertial i_r_uglut_r_lglut_r_uleg =
      i_r_uglut.GetInertial(math::Pose()) +
      i_r_lglut.GetInertial(r_lglut_r_uglut) +
      i_r_uleg.GetInertial(r_uleg_r_uglut);
    physics::Inertial i_r_uglut_r_lglut_r_uleg_mod =
      i_r_uglut_mod.GetInertial(math::Pose()) +
      i_r_lglut_mod.GetInertial(r_lglut_r_uglut) +
      i_r_uleg_mod.GetInertial(r_uleg_r_uglut);
    std::cout << "\nCheck IZZ of foot, talus and lleg at lleg(uay) link:\n"
          << "  original izz: " << i_r_uglut_r_lglut_r_uleg.GetIZZ() << "\n"
          << "  modified izz: " << i_r_uglut_r_lglut_r_uleg_mod.GetIZZ() << "\n"
          << "  changed  izz: " << i_r_uglut_r_lglut_r_uleg_mod.GetIZZ() -
                                   i_r_uglut_r_lglut_r_uleg.GetIZZ() << "\n"
          << "  changed% izz: " << (i_r_uglut_r_lglut_r_uleg_mod.GetIZZ() -
                                    i_r_uglut_r_lglut_r_uleg.GetIZZ()) /
                                    i_r_uglut_r_lglut_r_uleg.GetIZZ() << "\n";

    // modify IYY by splitting equally between pelvis, r_uglut, r_lglut
    //    this actually does not affect dynamics, because all three lins
    //    are fully constrained in y-axis.
    // We have to take more advantage of this types of inertia distribution.
    double iyy_pelvis2 = i_pelvis_mod2.GetIYY();
    double iyy_r_uglut = i_r_uglut_mod.GetIYY();
    double iyy_r_lglut = i_r_lglut_mod.GetIYY();
    i_r_uglut_mod.SetIYY((iyy_r_uglut + iyy_r_lglut + iyy_pelvis2) / 3.0);
    i_r_lglut_mod.SetIYY((iyy_r_uglut + iyy_r_lglut + iyy_pelvis2) / 3.0);
    i_pelvis_mod2.SetIYY((iyy_r_uglut + iyy_r_lglut + iyy_pelvis2) / 3.0);
    // check increase of IYY of r_uglut, r_lglut, pelvis about pelvis(lbz)
    // make sure it's reasonable
    math::Pose pelvis_r_uleg =
      r_uleg->GetWorldPose() - pelvis->GetWorldPose();
    math::Pose r_uglut_r_uleg =
      r_uleg->GetWorldPose() - r_uglut->GetWorldPose();
    math::Pose r_lglut_r_uleg =
      r_uleg->GetWorldPose() - r_lglut->GetWorldPose();
    physics::Inertial i_pelvis_r_uglut_r_lglut =
      i_r_uglut.GetInertial(r_uglut_r_uleg) +
      i_r_lglut.GetInertial(r_lglut_r_uleg) +
      i_pelvis.GetInertial(pelvis_r_uleg);
    physics::Inertial i_pelvis_r_uglut_r_lglut_mod =
      i_r_uglut_mod.GetInertial(r_uglut_r_uleg) +
      i_r_lglut_mod.GetInertial(r_lglut_r_uleg) +
      i_pelvis_mod.GetInertial(pelvis_r_uleg);
    std::cout << "\nCheck IYY of foot, talus and lleg at lleg(uay) link:\n"
          << "  original iyy: " << i_pelvis_r_uglut_r_lglut.GetIYY() << "\n"
          << "  modified iyy: " << i_pelvis_r_uglut_r_lglut_mod.GetIYY() << "\n"
          << "  changed  iyy: " << i_pelvis_r_uglut_r_lglut_mod.GetIYY() -
                                   i_pelvis_r_uglut_r_lglut.GetIYY() << "\n"
          << "  changed% iyy: " << (i_pelvis_r_uglut_r_lglut_mod.GetIYY() -
                                    i_pelvis_r_uglut_r_lglut.GetIYY()) /
                                    i_pelvis_r_uglut_r_lglut.GetIYY() << "\n";

    // distribute IXX across pelvis and uglut
    double ixx_pelvis2 = i_pelvis_mod2.GetIXX();
    double ixx_r_uglut = i_r_uglut_mod.GetIXX();
    i_pelvis_mod2.SetIXX((ixx_r_uglut + ixx_pelvis2) / 2.0);
    i_r_uglut_mod.SetIXX((ixx_r_uglut + ixx_pelvis2) / 2.0);
    // check increase of IXX of r_uglut, pelvis about lglut(mhx)
    // make sure it's not changed
    math::Pose r_uglut_r_lglut =
      r_lglut->GetWorldPose() - r_uglut->GetWorldPose();
    math::Pose pelvis_r_lglut =
      r_lglut->GetWorldPose() - pelvis->GetWorldPose();
    physics::Inertial i_pelvis_r_uglut =
      i_r_uglut.GetInertial(r_uglut_r_lglut) +
      i_pelvis.GetInertial(pelvis_r_lglut);
    physics::Inertial i_pelvis_r_uglut_mod =
      i_r_uglut_mod.GetInertial(r_uglut_r_lglut) +
      i_pelvis_mod.GetInertial(pelvis_r_lglut);
    std::cout << "\nCheck IXX of pelvis and uglut at lleg(mhx) link:\n"
          << "  original ixx: " << i_pelvis_r_uglut.GetIXX() << "\n"
          << "  modified ixx: " << i_pelvis_r_uglut_mod.GetIXX() << "\n"
          << "  changed  ixx: " << i_pelvis_r_uglut_mod.GetIXX() -
                                   i_pelvis_r_uglut.GetIXX() << "\n"
          << "  changed% ixx: " << (i_pelvis_r_uglut_mod.GetIXX() -
                                    i_pelvis_r_uglut.GetIXX()) /
                                    i_pelvis_r_uglut.GetIXX() << "\n";

    // distribute IXX across lglut and uleg
    double ixx_r_lglut = i_r_lglut_mod.GetIXX();
    double ixx_r_uleg = i_r_uleg_mod.GetIXX();
    i_r_lglut_mod.SetIXX((ixx_r_uleg + ixx_r_lglut) / 2.0);
    i_r_uleg_mod.SetIXX((ixx_r_uleg + ixx_r_lglut) / 2.0);
    // check increase of IXX of r_uleg, r_lglut about lglut(mhx)
    // make sure it's not changed
    math::Pose r_uleg_r_lglut =
      r_lglut->GetWorldPose() - r_uleg->GetWorldPose();
    physics::Inertial i_r_lglut_r_uleg =
      i_r_uleg.GetInertial(r_uleg_r_lglut) +
      i_r_lglut.GetInertial(math::Pose());
    physics::Inertial i_r_lglut_r_uleg_mod =
      i_r_uleg_mod.GetInertial(r_uleg_r_lglut) +
      i_r_lglut_mod.GetInertial(math::Pose());
    std::cout << "\nCheck IXX of lglut and uleg at lglut(mhx) link:\n"
          << "  original ixx: " << i_r_lglut_r_uleg.GetIXX() << "\n"
          << "  modified ixx: " << i_r_lglut_r_uleg_mod.GetIXX() << "\n"
          << "  changed  ixx: " << i_r_lglut_r_uleg_mod.GetIXX() -
                                   i_r_lglut_r_uleg.GetIXX() << "\n"
          << "  changed% ixx: " << (i_r_lglut_r_uleg_mod.GetIXX() -
                                    i_r_lglut_r_uleg.GetIXX()) /
                                    i_r_lglut_r_uleg.GetIXX() << "\n";

    // what's the new inertial
    std::cout << "\n============ Gluts Summary ==============\n";
    std::cout << "original pelvis Mass@cog\n" << i_pelvis.GetMass() << "\n";
    std::cout << "original pelvis MOI@cog\n" << i_pelvis.GetMOI() << "\n";
    std::cout << "modified pelvis Mass@cog\n" << i_pelvis_mod2.GetMass()<< "\n";
    std::cout << "modified pelvis MOI@cog\n" << i_pelvis_mod2.GetMOI()<< "\n";
    std::cout << "original r_uglut Mass@cog\n" << i_r_uglut.GetMass() << "\n";
    std::cout << "original r_uglut MOI@cog\n" << i_r_uglut.GetMOI() << "\n";
    std::cout << "modified r_uglut Mass@cog\n" << i_r_uglut_mod.GetMass()<<"\n";
    std::cout << "modified r_uglut MOI@cog\n" << i_r_uglut_mod.GetMOI() << "\n";
    std::cout << "original r_lglut Mass@cog\n" << i_r_lglut.GetMass() << "\n";
    std::cout << "original r_lglut MOI@cog\n" << i_r_lglut.GetMOI() << "\n";
    std::cout << "modified r_lglut Mass@cog\n" << i_r_lglut_mod.GetMass()<<"\n";
    std::cout << "modified r_lglut MOI@cog\n" << i_r_lglut_mod.GetMOI() << "\n";
    std::cout << "original r_uleg Mass@cog\n" << i_r_uleg.GetMass() << "\n";
    std::cout << "original r_uleg MOI@cog\n" << i_r_uleg.GetMOI() << "\n";
    std::cout << "modified r_uleg Mass@cog\n" << i_r_uleg_mod.GetMass() << "\n";
    std::cout << "modified r_uleg MOI@cog\n" << i_r_uleg_mod.GetMOI() << "\n";

    // sanity check: build aggregate inertia of robot viewed from ankle up
    physics::Link_V links = this->model->GetLinks();
    physics::Inertial totalUAYUp;
    for(physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
    {
      if ((*li)->GetName() != "l_foot" && (*li)->GetName() != "l_talus" &&
          (*li)->GetName() != "r_foot" && (*li)->GetName() != "r_talus" )
      {
        math::Pose linkToUAY = r_talus->GetWorldPose() - (*li)->GetWorldPose();
        // gzerr << "accumulating: " <<  (*li)->GetName()
        //       << " inertia\n" << (*li)->GetInertial()->GetInertial(linkToUAY)
        //       << "\n";
        totalUAYUp = totalUAYUp + (*li)->GetInertial()->GetInertial(linkToUAY);
      }
      else
      {
        // gzerr << "skipping: " <<  (*li)->GetName() << "\n";
      }
        
    }
    std::cout << "\n============ Total Inertia of Atlas above ankle from ankle"
          << " ==============\n";
    std::cout << "original total Mass@cog\n" << totalUAYUp.GetMass() << "\n";
    std::cout << "original total MOI@cog\n" << totalUAYUp.GetMOI() << "\n";
    std::cout << "modified pelvis Mass@cog\n" << i_pelvis_mod2.GetMass()<< "\n";
    std::cout << "modified pelvis MOI@cog\n" << i_pelvis_mod2.GetMOI()<< "\n";
 
  }

  // JointController: Publish messages to reset joint controller gains
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    msgs::JointCmd msg;
    msg.set_name(this->joints[i]->GetScopedName()); 
    msg.mutable_position()->set_target(0.0);
    msg.mutable_position()->set_p_gain(0.0);
    msg.mutable_position()->set_i_gain(0.0);
    msg.mutable_position()->set_d_gain(0.0);
    msg.mutable_position()->set_i_max(0.0);
    msg.mutable_position()->set_i_min(0.0);
    msg.mutable_position()->set_limit(0.0);
  }

  this->errorTerms.resize(this->joints.size());
  for (unsigned i = 0; i < this->errorTerms.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].k_i_q_i = 0;
    this->errorTerms[i].qd_p = 0;
  }

  this->effortLimit.resize(this->jointNames.size());
  for (unsigned i = 0; i < this->effortLimit.size(); ++i)
    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);

  // We are not sending names due to the fact that there is an enum
  // joint indices in AtlasState.msg.
  this->atlasState.joint_states.name.clear();
  this->atlasState.joint_states.position.resize(this->joints.size());
  this->atlasState.joint_states.velocity.resize(this->joints.size());
  this->atlasState.joint_states.effort.resize(this->joints.size());
  this->atlasState.joint_states.kp_position.resize(this->joints.size());
  this->atlasState.joint_states.ki_position.resize(this->joints.size());
  this->atlasState.joint_states.kd_position.resize(this->joints.size());
  this->atlasState.joint_states.kp_velocity.resize(this->joints.size());
  this->atlasState.joint_states.i_effort_min.resize(this->joints.size());
  this->atlasState.joint_states.i_effort_max.resize(this->joints.size());

  // Setup jointStates: downside of using JointCommands in AtlasState
  // is that we have to maintain a copy of JointState locally and
  // memcopy all the arrays.  If we switch to JointState inside
  // AtlasState, the need of jointStates variable can be avoided,
  // but no access to PID gains.
  this->jointStates.name.resize(this->joints.size());
  this->jointStates.position.resize(this->joints.size());
  this->jointStates.velocity.resize(this->joints.size());
  this->jointStates.effort.resize(this->joints.size());

  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->jointStates.name[i] = this->jointNames[i];

  this->jointCommands.name.resize(this->joints.size());
  this->jointCommands.position.resize(this->joints.size());
  this->jointCommands.velocity.resize(this->joints.size());
  this->jointCommands.effort.resize(this->joints.size());
  this->jointCommands.kp_position.resize(this->joints.size());
  this->jointCommands.ki_position.resize(this->joints.size());
  this->jointCommands.kd_position.resize(this->joints.size());
  this->jointCommands.kp_velocity.resize(this->joints.size());
  this->jointCommands.i_effort_min.resize(this->joints.size());
  this->jointCommands.i_effort_max.resize(this->joints.size());

  this->ZeroJointCommands();

  // AtlasSimInterface:  initialize toRobot
  this->toRobot.timestamp = 1.0e9 * this->world->GetSimTime().nsec
    + this->world->GetSimTime().nsec;
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->toRobot.j[i].q_d = 0;
    this->toRobot.j[i].qd_d = 0;
    this->toRobot.j[i].f_d = 0;
    this->toRobot.jparams[i].k_q_p = 0;
    this->toRobot.jparams[i].k_q_i = 0;
    this->toRobot.jparams[i].k_qd_p = 0;
  }

  // AtlasSimInterface:  initialize fromRobot joints data
  this->fromRobot.t = this->world->GetSimTime().Double();
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->fromRobot.j[i].q = 0;
    this->fromRobot.j[i].qd = 0;
    this->fromRobot.j[i].f = 0;
    /*
    this->fromRobot.error_terms[i].q_p = 0;
    this->fromRobot.error_terms[i].qd_p = 0;
    this->fromRobot.error_terms[i].k_i_q_i = 0;
    */
  }

  // AtlasSimInterface:  initialize fromRobot sensor data
  this->fromRobot.imu.imu_timestamp = this->toRobot.timestamp;
  this->fromRobot.imu.angular_velocity.n[0] = 0;
  this->fromRobot.imu.angular_velocity.n[1] = 0;
  this->fromRobot.imu.angular_velocity.n[2] = 0;
  this->fromRobot.imu.linear_acceleration.n[0] = 0;
  this->fromRobot.imu.linear_acceleration.n[1] = 0;
  this->fromRobot.imu.linear_acceleration.n[2] = 0;
  this->fromRobot.imu.orientation_estimate.m_qw = 0;
  this->fromRobot.imu.orientation_estimate.m_qx = 0;
  this->fromRobot.imu.orientation_estimate.m_qy = 0;
  this->fromRobot.imu.orientation_estimate.m_qz = 0;
  this->fromRobot.foot_sensors[0].fz = 0;
  this->fromRobot.foot_sensors[0].mx = 0;
  this->fromRobot.foot_sensors[0].my = 0;
  this->fromRobot.foot_sensors[1].fz = 0;
  this->fromRobot.foot_sensors[1].mx = 0;
  this->fromRobot.foot_sensors[1].my = 0;
  this->fromRobot.wrist_sensors[0].f.n[0] = 0;
  this->fromRobot.wrist_sensors[0].f.n[1] = 0;
  this->fromRobot.wrist_sensors[0].f.n[2] = 0;
  this->fromRobot.wrist_sensors[0].m.n[0] = 0;
  this->fromRobot.wrist_sensors[0].m.n[1] = 0;
  this->fromRobot.wrist_sensors[0].m.n[2] = 0;
  this->fromRobot.wrist_sensors[1].f.n[0] = 0;
  this->fromRobot.wrist_sensors[1].f.n[1] = 0;
  this->fromRobot.wrist_sensors[1].f.n[2] = 0;
  this->fromRobot.wrist_sensors[1].m.n[0] = 0;
  this->fromRobot.wrist_sensors[1].m.n[1] = 0;
  this->fromRobot.wrist_sensors[1].m.n[2] = 0;
  // internal debugging use only
  this->fromRobot.pelvis_position.n[0] = 0;
  this->fromRobot.pelvis_position.n[1] = 0;
  this->fromRobot.pelvis_position.n[2] = 0;
  this->fromRobot.pelvis_velocity.n[0] = 0;
  this->fromRobot.pelvis_velocity.n[1] = 0;
  this->fromRobot.pelvis_velocity.n[2] = 0;

  // AtlasSimInterface:
  // Calling into the behavior library to reset controls and set startup
  // behavior.
  this->errorCode = this->atlasSimInterface->reset_control();
  this->errorCode = this->atlasSimInterface->set_desired_behavior("safety");

  // Get imu link
  this->imuLink = this->model->GetLink(this->imuLinkName);
  if (!this->imuLink)
    gzerr << this->imuLinkName << " not found\n";

  // AtlasSimInterface: Get pelvis link for internal debugging only
  this->pelvisLink = this->model->GetLink(this->pelvisLinkName);
  if (!this->pelvisLink)
    gzerr << this->pelvisLinkName << " not found\n";

  // Get force torque joints
  this->lWristJoint = this->model->GetJoint("l_arm_mwx");
  if (!this->lWristJoint)
    gzerr << "left wrist joint (l_arm_mwx) not found\n";

  this->rWristJoint = this->model->GetJoint("r_arm_mwx");
  if (!this->rWristJoint)
    gzerr << "right wrist joint (r_arm_mxw) not found\n";

  this->rAnkleJoint = this->model->GetJoint("r_leg_lax");
  if (!this->rAnkleJoint)
    gzerr << "right ankle joint (r_leg_lax) not found\n";

  this->lAnkleJoint = this->model->GetJoint("l_leg_lax");
  if (!this->lAnkleJoint)
    gzerr << "left ankle joint (l_leg_lax) not found\n";

  // Get sensors
  this->imuSensor =
    boost::shared_dynamic_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::pelvis::"
        "imu_sensor"));
  if (!this->imuSensor)
    gzerr << "imu_sensor not found\n" << "\n";

  this->rFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::r_foot::"
        "r_foot_contact_sensor"));
  if (!this->rFootContactSensor)
    gzerr << "r_foot_contact_sensor not found\n" << "\n";

  this->lFootContactSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->model->GetScopedName()
        + "::l_foot::"
        "l_foot_contact_sensor"));
  if (!this->lFootContactSensor)
    gzerr << "l_foot_contact_sensor not found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&AtlasPlugin::DeferredLoad, this));
}


////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->UpdateJointCommands(*_msg);
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::UpdateJointCommands(const osrf_msgs::JointCommands &_msg)
{
  this->jointCommands.header.stamp = _msg.header.stamp;

  if (_msg.position.size() == this->jointCommands.position.size())
    std::copy(_msg.position.begin(), _msg.position.end(),
      this->jointCommands.position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg.position.size(), this->jointCommands.position.size());

  if (_msg.velocity.size() == this->jointCommands.velocity.size())
    std::copy(_msg.velocity.begin(), _msg.velocity.end(),
      this->jointCommands.velocity.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg.velocity.size(), this->jointCommands.velocity.size());

  if (_msg.effort.size() == this->jointCommands.effort.size())
    std::copy(_msg.effort.begin(), _msg.effort.end(),
      this->jointCommands.effort.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg.effort.size(), this->jointCommands.effort.size());

  if (_msg.kp_position.size() == this->jointCommands.kp_position.size())
    std::copy(_msg.kp_position.begin(), _msg.kp_position.end(),
      this->jointCommands.kp_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg.kp_position.size(), this->jointCommands.kp_position.size());

  if (_msg.ki_position.size() == this->jointCommands.ki_position.size())
    std::copy(_msg.ki_position.begin(), _msg.ki_position.end(),
      this->jointCommands.ki_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg.ki_position.size(), this->jointCommands.ki_position.size());

  if (_msg.kd_position.size() == this->jointCommands.kd_position.size())
    std::copy(_msg.kd_position.begin(), _msg.kd_position.end(),
      this->jointCommands.kd_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg.kd_position.size(), this->jointCommands.kd_position.size());

  if (_msg.kp_velocity.size() == this->jointCommands.kp_velocity.size())
    std::copy(_msg.kp_velocity.begin(), _msg.kp_velocity.end(),
      this->jointCommands.kp_velocity.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg.kp_velocity.size(), this->jointCommands.kp_velocity.size());

  if (_msg.i_effort_min.size() == this->jointCommands.i_effort_min.size())
    std::copy(_msg.i_effort_min.begin(), _msg.i_effort_min.end(),
      this->jointCommands.i_effort_min.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg.i_effort_min.size(), this->jointCommands.i_effort_min.size());

  if (_msg.i_effort_max.size() == this->jointCommands.i_effort_max.size())
    std::copy(_msg.i_effort_max.begin(), _msg.i_effort_max.end(),
      this->jointCommands.i_effort_max.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg.i_effort_max.size(), this->jointCommands.i_effort_max.size());
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // Get window size from ros parameter server (seconds)
  if (!this->rosNode->getParam(
    "atlas_controller/statistics_time_window_size",
    this->jointCommandsAgeBufferDuration))
  {
    this->jointCommandsAgeBufferDuration = 1.0;
    ROS_INFO("controller statistics window size not specified in"
             " ros parameter server, defaulting to %f sec.",
             this->jointCommandsAgeBufferDuration);
  }
  double stepSize = this->world->GetPhysicsEngine()->GetStepTime();
  if (math::equal(stepSize, 0.0))
  {
    stepSize = 0.001;
    ROS_WARN("simulation step size is zero, something is wrong,"
              "  Defaulting to step size of %f sec.", stepSize);
  }
  // document this from
  // http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  // Online algorithm
  // where Delta2 buffer contains delta*(x - mean) line from code block
  unsigned int bufferSize = this->jointCommandsAgeBufferDuration / stepSize;
  this->jointCommandsAgeBuffer.resize(bufferSize);
  this->jointCommandsAgeDelta2Buffer.resize(bufferSize);
  this->jointCommandsAgeBufferIndex = 0;
  this->jointCommandsAgeMean = 0.0;
  this->jointCommandsAgeVariance = 0.0;

  // ROS Controller API
  /// brief broadcasts the robot states
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "atlas/joint_states", 1);

  this->pubAtlasState = this->rosNode->advertise<atlas_msgs::AtlasState>(
    "atlas/atlas_states", 1);

  this->pubForceTorqueSensors =
    this->rosNode->advertise<atlas_msgs::ForceTorqueSensors>(
    "atlas/force_torque_sensors", 10);

  // ros publication / subscription
  this->pubControllerStatistics =
    this->rosNode->advertise<atlas_msgs::ControllerStatistics>(
    "atlas/controller_statistics", 10);

  // these topics are used for debugging only
  this->pubLFootContact =
    this->rosNode->advertise<geometry_msgs::WrenchStamped>(
      "atlas/debug/l_foot_contact", 10);

  // these topics are used for debugging only
  this->pubRFootContact =
    this->rosNode->advertise<geometry_msgs::WrenchStamped>(
      "atlas/debug/r_foot_contact", 10);

  // ros topic subscribtions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "atlas/joint_commands", 1,
    boost::bind(&AtlasPlugin::SetJointCommands, this, _1),
    ros::VoidPtr(), &this->rosQueue);

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint commands, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointCommandsSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  this->subJointCommands =
    this->rosNode->subscribe(jointCommandsSo);

  // ros topic subscribtions
  ros::SubscribeOptions testSo =
    ros::SubscribeOptions::create<atlas_msgs::Test>(
    "atlas/debug/test", 1,
    boost::bind(&AtlasPlugin::SetExperimentalDampingPID, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subTest = this->rosNode->subscribe(testSo);

  // publish imu data
  this->pubImu =
    this->rosNode->advertise<sensor_msgs::Imu>("atlas/imu", 10);

  // initialize status pub time
  this->lastControllerStatisticsTime = this->world->GetSimTime().Double();

  // controller statistics update rate defaults to 1kHz,
  // read from ros param if available
  double rate;
  if (this->rosNode->getParam("atlas/controller_statistics/update_rate",
    rate))
  {
    rate = math::clamp(rate, 1.0, 10000.0);
    ROS_INFO("AtlasPlugin controller statistics %f kHz", rate);
    this->statsUpdateRate = rate;
  }
  else
  {
    ROS_INFO("AtlasPlugin default controller statistics 1kHz");
    this->statsUpdateRate = 1000.0;
  }

  // AtlasSimInterface:
  // subscribe to a control_mode string message, current valid commands are:
  //   walk, stand, safety, stand-prep, none
  // the command is passed to the AtlasSimInterface library.
  ros::SubscribeOptions atlasControlModeSo =
    ros::SubscribeOptions::create<std_msgs::String>(
    "atlas/control_mode", 100,
    boost::bind(&AtlasPlugin::OnRobotMode, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subAtlasControlMode = this->rosNode->subscribe(atlasControlModeSo);

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&AtlasPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&AtlasPlugin::UpdateStates, this));

  // on contact
  this->lContactUpdateConnection = this->lFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnLContactUpdate, this));

  this->rContactUpdateConnection = this->rFootContactSensor->ConnectUpdated(
     boost::bind(&AtlasPlugin::OnRContactUpdate, this));

  // Advertise services on the custom queue
  ros::AdvertiseServiceOptions resetControlsAso =
    ros::AdvertiseServiceOptions::create<atlas_msgs::ResetControls>(
      "atlas/reset_controls", boost::bind(
        &AtlasPlugin::ResetControls, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);
  this->resetControlsService = this->rosNode->advertiseService(
    resetControlsAso);
}

////////////////////////////////////////////////////////////////////////////////
bool AtlasPlugin::ResetControls(atlas_msgs::ResetControls::Request &_req,
  atlas_msgs::ResetControls::Response &_res)
{
  boost::mutex::scoped_lock lock(this->mutex);

  for (unsigned i = 0; i < this->errorTerms.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].k_i_q_i = 0;
    this->errorTerms[i].qd_p = 0;
  }

  this->UpdateJointCommands(_req.joint_commands);

  _res.success = true;
  _res.status_message = "success";
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// AtlasSimInterface:
// subscribe to a control_mode string message, current valid commands are:
//   walk, stand, safety, stand-prep, none
// the command is passed to the AtlasSimInterface library.
void AtlasPlugin::OnRobotMode(const std_msgs::String::ConstPtr &_mode)
{
  // to make it stand
  //  * stand-prep:  puts robot in standing pose while harnessed
  //  * remove the harness
  //  * after robot hits ground, switch over to stand mode
  //  * robot should dynamically balance itself

  // simple state machine here to do something
  if (_mode->data == "safety" || _mode->data == "stand-prep" ||
      _mode->data == "stand" || _mode->data == "walk")
  {
    // start AtlasSimLibrary controller
    // this mode resets the timer, and automatically goes into stand mode
    // after 
    this->usingWalkingController = true;
    this->atlasSimInterface->set_desired_behavior(_mode->data);
    this->ZeroJointCommands();
  }
  else if (_mode->data == "none")
  {
    // revert to PID control
    this->LoadPIDGainsFromParameter();
    this->usingWalkingController = false;
    this->atlasSimInterface->set_desired_behavior(_mode->data);
  }
  else
  {
    ROS_WARN("Unknown robot mode [%s]", _mode->data.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    double dt = (curTime - this->lastControllerUpdateTime).Double();
    static double e = this->timeConstant / (this->timeConstant + dt);

    // AtlasSimInterface:
    // populate fromRobot from robot
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->fromRobot.t = curTime.Double();
      this->fromRobot.j[i].q = this->joints[i]->GetAngle(0).Radian();
      this->fromRobot.j[i].qd = this->joints[i]->GetVelocity(0);
      // wait to fill in this->fromRobot.j[i].f later
    }

    // get imu data from imu link
    if (this->imuSensor && curTime > this->lastImuTime)
    {
      sensor_msgs::Imu* imuMsg = &this->atlasState.imu;
      imuMsg->header.frame_id = this->imuLinkName;
      imuMsg->header.stamp = ros::Time(curTime.Double());

      // compute angular rates
      {
        static math::Vector3 wLocal;
        if (math::equal(e, 0.0))
          wLocal = this->imuSensor->GetAngularVelocity();
        else
          wLocal = e*wLocal + (1.0 - e)*this->imuSensor->GetAngularVelocity();
        imuMsg->angular_velocity.x = wLocal.x;
        imuMsg->angular_velocity.y = wLocal.y;
        imuMsg->angular_velocity.z = wLocal.z;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.angular_velocity.n[0] = wLocal.x;
        this->fromRobot.imu.angular_velocity.n[1] = wLocal.y;
        this->fromRobot.imu.angular_velocity.n[2] = wLocal.z;
      }

      // compute acceleration
      {
        static math::Vector3 accel;
        if (math::equal(e, 0.0))
          accel = this->imuSensor->GetLinearAcceleration();
        else
          accel = e*accel + (1.0 - e)*this->imuSensor->GetLinearAcceleration();
        imuMsg->linear_acceleration.x = accel.x;
        imuMsg->linear_acceleration.y = accel.y;
        imuMsg->linear_acceleration.z = accel.z;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.linear_acceleration.n[0] = accel.x;
        this->fromRobot.imu.linear_acceleration.n[1] = accel.y;
        this->fromRobot.imu.linear_acceleration.n[2] = accel.z;
      }

      // compute orientation
      {
        static math::Quaternion imuRot;
        if (math::equal(e, 0.0))
          imuRot = this->imuSensor->GetOrientation();
        else
          imuRot = imuRot*e + this->imuSensor->GetOrientation()*(1.0 - e);
        // imuRot = e*imuRot + (1.0 - e)*this->imuSensor->GetOrientation();
        imuMsg->orientation.x = imuRot.x;
        imuMsg->orientation.y = imuRot.y;
        imuMsg->orientation.z = imuRot.z;
        imuMsg->orientation.w = imuRot.w;

        // AtlasSimInterface: populate imu in fromRobot
        this->fromRobot.imu.orientation_estimate.m_qw = imuRot.w;
        this->fromRobot.imu.orientation_estimate.m_qx = imuRot.x;
        this->fromRobot.imu.orientation_estimate.m_qy = imuRot.y;
        this->fromRobot.imu.orientation_estimate.m_qz = imuRot.z;
      }

      this->pubImu.publish(*imuMsg);

      // update time
      this->lastImuTime = curTime.Double();
    }

    // AtlasSimInterface: pelvis pose/twist for internal debugging only.
    if (this->pelvisLink)
    {
      math::Pose pose = this->pelvisLink->GetWorldPose();
      math::Vector3 vel = this->pelvisLink->GetWorldLinearVel();

      /// WARNING: these are inertial?
      this->fromRobot.pelvis_position.n[0] = pose.pos.x;
      this->fromRobot.pelvis_position.n[1] = pose.pos.y;
      this->fromRobot.pelvis_position.n[2] = pose.pos.z;
      this->fromRobot.pelvis_velocity.n[0] = vel.x;
      this->fromRobot.pelvis_velocity.n[1] = vel.y;
      this->fromRobot.pelvis_velocity.n[2] = vel.z;
    }

    atlas_msgs::ForceTorqueSensors* forceTorqueSensorsMsg =
      &this->atlasState.force_torque_sensors;

    forceTorqueSensorsMsg->header.stamp =
      ros::Time(curTime.sec, curTime.nsec);

    // The following is added to fix compiler warnings.
    unsigned int i0 = 0;

    // get force torque at left ankle and publish
    if (this->lAnkleJoint)
    {
      static physics::JointWrench wrench;
      // if (math::equal(e, 0.0))
        wrench = this->lAnkleJoint->GetForceTorque(i0);
      // else
      //   wrench = e*wrench + (1.0 - e)*this->lAnkleJoint->GetForceTorque(i0);
      forceTorqueSensorsMsg->l_foot.force.z  = wrench.body1Force.z;
      forceTorqueSensorsMsg->l_foot.torque.x = wrench.body1Torque.x;
      forceTorqueSensorsMsg->l_foot.torque.y = wrench.body1Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in fromRobot
      this->fromRobot.foot_sensors[0].fz =
        forceTorqueSensorsMsg->l_foot.force.z;
      this->fromRobot.foot_sensors[0].mx =
        forceTorqueSensorsMsg->l_foot.torque.x;
      this->fromRobot.foot_sensors[0].my =
        forceTorqueSensorsMsg->l_foot.torque.y;
    }

    // get force torque at right ankle and publish
    if (this->rAnkleJoint)
    {
      static physics::JointWrench wrench;
      wrench = this->rAnkleJoint->GetForceTorque(i0);
      // wrench = e*wrench + (1.0 - e)*this->rAnkleJoint->GetForceTorque(i0);
      forceTorqueSensorsMsg->r_foot.force.z = wrench.body1Force.z;
      forceTorqueSensorsMsg->r_foot.torque.x = wrench.body1Torque.x;
      forceTorqueSensorsMsg->r_foot.torque.y = wrench.body1Torque.y;

      // AtlasSimInterface: populate foot force torque sensor in fromRobot
      this->fromRobot.foot_sensors[1].fz =
        forceTorqueSensorsMsg->r_foot.force.z;
      this->fromRobot.foot_sensors[1].mx =
        forceTorqueSensorsMsg->r_foot.torque.x;
      this->fromRobot.foot_sensors[1].my =
        forceTorqueSensorsMsg->r_foot.torque.y;
    }

    // get force torque at left wrist and publish
    if (this->lWristJoint)
    {
      static physics::JointWrench wrench;
      wrench = this->lWristJoint->GetForceTorque(i0);
      // wrench = e*wrench + (1.0 - e)*this->lWristJoint->GetForceTorque(i0);
      forceTorqueSensorsMsg->l_hand.force.x  = wrench.body1Force.x;
      forceTorqueSensorsMsg->l_hand.force.y  = wrench.body1Force.y;
      forceTorqueSensorsMsg->l_hand.force.z  = wrench.body1Force.z;
      forceTorqueSensorsMsg->l_hand.torque.x = wrench.body1Torque.x;
      forceTorqueSensorsMsg->l_hand.torque.y = wrench.body1Torque.y;
      forceTorqueSensorsMsg->l_hand.torque.z = wrench.body1Torque.z;

      // AtlasSimInterface: populate wrist force torque sensor in fromRobot
      this->fromRobot.wrist_sensors[0].f.n[0] =
        forceTorqueSensorsMsg->l_hand.force.x;
      this->fromRobot.wrist_sensors[0].f.n[1] =
        forceTorqueSensorsMsg->l_hand.force.y;
      this->fromRobot.wrist_sensors[0].f.n[2] =
        forceTorqueSensorsMsg->l_hand.force.z;
      this->fromRobot.wrist_sensors[0].m.n[0] =
        forceTorqueSensorsMsg->l_hand.torque.x;
      this->fromRobot.wrist_sensors[0].m.n[1] =
        forceTorqueSensorsMsg->l_hand.torque.y;
      this->fromRobot.wrist_sensors[0].m.n[2] =
        forceTorqueSensorsMsg->l_hand.torque.z;
    }

    // get force torque at right wrist and publish
    if (this->rWristJoint)
    {
      static physics::JointWrench wrench;
      wrench = this->rWristJoint->GetForceTorque(i0);
      // wrench = e*wrench + (1.0 - e)*this->rWristJoint->GetForceTorque(i0);
      forceTorqueSensorsMsg->r_hand.force.x  = wrench.body1Force.x;
      forceTorqueSensorsMsg->r_hand.force.y  = wrench.body1Force.y;
      forceTorqueSensorsMsg->r_hand.force.z  = wrench.body1Force.z;
      forceTorqueSensorsMsg->r_hand.torque.x = wrench.body1Torque.x;
      forceTorqueSensorsMsg->r_hand.torque.y = wrench.body1Torque.y;
      forceTorqueSensorsMsg->r_hand.torque.z = wrench.body1Torque.z;

      // AtlasSimInterface: populate wrist force torque sensor in fromRobot
      this->fromRobot.wrist_sensors[1].f.n[0] =
        forceTorqueSensorsMsg->r_hand.force.x;
      this->fromRobot.wrist_sensors[1].f.n[1] =
        forceTorqueSensorsMsg->r_hand.force.y;
      this->fromRobot.wrist_sensors[1].f.n[2] =
        forceTorqueSensorsMsg->r_hand.force.z;
      this->fromRobot.wrist_sensors[1].m.n[0] =
        forceTorqueSensorsMsg->r_hand.torque.x;
      this->fromRobot.wrist_sensors[1].m.n[1] =
        forceTorqueSensorsMsg->r_hand.torque.y;
      this->fromRobot.wrist_sensors[1].m.n[2] =
        forceTorqueSensorsMsg->r_hand.torque.z;
    }
    this->pubForceTorqueSensors.publish(*forceTorqueSensorsMsg);

    // populate atlasState from robot
    this->atlasState.header.stamp = ros::Time(curTime.sec, curTime.nsec);

    // populate jointStates from robot both for atlas_states and joint_states
    this->atlasState.joint_states.header.stamp =
      this->jointStates.header.stamp;
    this->jointStates.header.stamp = this->atlasState.header.stamp;

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->atlasState.joint_states.position[i] =
        this->joints[i]->GetAngle(0).Radian();
      this->atlasState.joint_states.velocity[i] =
        this->joints[i]->GetVelocity(0);
    }
    // copy from atlasState.joint_states.position into joint_states.position
    GZ_ASSERT(this->atlasState.joint_states.position.size() ==
              this->jointStates.position.size(),
              "atlasState.joint_states.position and "
              "jointStates.position size mismatch.");
    std::copy(this->atlasState.joint_states.position.begin(),
              this->atlasState.joint_states.position.end(),
              this->jointStates.position.begin());
    // copy from atlasState.joint_states.velocity into joint_states.velocity
    GZ_ASSERT(this->atlasState.joint_states.velocity.size() ==
              this->jointStates.velocity.size(),
              "atlasState.joint_states.velocity and "
              "jointStates.velocity size mismatch.");
    std::copy(this->atlasState.joint_states.velocity.begin(),
              this->atlasState.joint_states.velocity.end(),
              this->jointStates.velocity.begin());

    // AtlasSimInterface:
    if (this->usingWalkingController)
      // process data fromRobot to create output data toRobot
      this->errorCode = this->atlasSimInterface->process_control_input(
        this->fromRobot, this->toRobot);

    {
      boost::mutex::scoped_lock lock(this->mutex);
      {
        // Keep track of age of jointCommands age in seconds.
        // Note the value is invalid as a moving window average age
        // until the buffer is full.
        this->jointCommandsAge = curTime.Double() -
          this->jointCommands.header.stamp.toSec();

        double weightedJointCommandsAge = this->jointCommandsAge
          / this->jointCommandsAgeBuffer.size();

        // for variance calculation, save delta before average is updated.
        double delta = this->jointCommandsAge - this->jointCommandsAgeMean;

        // update average
        this->jointCommandsAgeMean += weightedJointCommandsAge;
        this->jointCommandsAgeMean -=
          this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex];

        // update variance with new average
        double delta2 = delta *
          (this->jointCommandsAge - this->jointCommandsAgeMean);
        this->jointCommandsAgeVariance += delta2;
        this->jointCommandsAgeVariance -=
          this->jointCommandsAgeDelta2Buffer[
          this->jointCommandsAgeBufferIndex];

        // save weighted average in window
        this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex] =
          weightedJointCommandsAge;

        // save delta buffer for incremental variance calculation
        this->jointCommandsAgeDelta2Buffer[
          this->jointCommandsAgeBufferIndex] = delta2;

        this->jointCommandsAgeBufferIndex =
         (this->jointCommandsAgeBufferIndex + 1) %
         this->jointCommandsAgeBuffer.size();
      }

      /// update pid with feedforward force
      for (unsigned int i = 0; i < this->joints.size(); ++i)
      {
        // truncate joint position within range of motion
        double positionTarget = math::clamp(
          this->jointCommands.position[i],
          this->joints[i]->GetLowStop(0).Radian(),
          this->joints[i]->GetHighStop(0).Radian());

        double q_p = positionTarget -
          this->atlasState.joint_states.position[i];

        if (!math::equal(dt, 0.0))
          this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

        this->errorTerms[i].q_p = q_p;

        this->errorTerms[i].qd_p =
          this->jointCommands.velocity[i] -
          this->atlasState.joint_states.velocity[i];

        this->errorTerms[i].k_i_q_i = math::clamp(
          this->errorTerms[i].k_i_q_i +
          dt * this->jointCommands.ki_position[i] * this->errorTerms[i].q_p,
          static_cast<double>(this->jointCommands.i_effort_min[i]),
          static_cast<double>(this->jointCommands.i_effort_max[i]));

        // use gain params to compute force cmd
        double forceUnclamped =
          this->jointCommands.kp_position[i] * this->errorTerms[i].q_p +
                                               this->errorTerms[i].k_i_q_i +
          this->jointCommands.kd_position[i] * this->errorTerms[i].d_q_p_dt +
          this->jointCommands.kp_velocity[i] * this->errorTerms[i].qd_p +
          this->jointCommands.effort[i];

        // keep unclamped force for integral tie-back calculation
        double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i],
          this->effortLimit[i]);

        // integral tie-back during control saturation if using integral gain
        if (!math::equal(forceClamped,forceUnclamped) &&
            !math::equal(this->jointCommands.ki_position[i],0.0) )
        {
          // lock integral term to provide continuous control as system moves
          // out of staturation
          this->errorTerms[i].k_i_q_i = math::clamp(
            this->errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
          static_cast<double>(this->jointCommands.i_effort_min[i]),
          static_cast<double>(this->jointCommands.i_effort_max[i]));
        }

        // AtlasSimInterface:  add controller feed forward force
        // to overall control torque.
        forceClamped = math::clamp(forceUnclamped + this->toRobot.j[i].f_d,
          -this->effortLimit[i], this->effortLimit[i]);

        this->joints[i]->SetForce(0, forceClamped);

        // fill in jointState efforts
        this->atlasState.joint_states.effort[i] = forceClamped;

        // AtlasSimInterface: fill in fromRobot efforts.
        // FIXME: Is this used by the controller?  i.e. should this happen
        // before process_control_input?
        this->fromRobot.j[i].f = forceClamped;
      }
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.kp_position.size() ==
                this->atlasState.joint_states.kp_position.size(),
                "jointCommands.kp_position and "
                "atlasState.joint_states.kp_position size mismatch.");
      std::copy(this->jointCommands.kp_position.begin(),
                this->jointCommands.kp_position.end(),
                this->atlasState.joint_states.kp_position.begin());
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.ki_position.size() ==
                this->atlasState.joint_states.ki_position.size(),
                "jointCommands.ki_position and "
                "atlasState.joint_states.ki_position size mismatch.");
      std::copy(this->jointCommands.ki_position.begin(),
                this->jointCommands.ki_position.end(),
                this->atlasState.joint_states.ki_position.begin());
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.kd_position.size() ==
                this->atlasState.joint_states.kd_position.size(),
                "jointCommands.kd_position and "
                "atlasState.joint_states.kd_position size mismatch.");
      std::copy(this->jointCommands.kd_position.begin(),
                this->jointCommands.kd_position.end(),
                this->atlasState.joint_states.kd_position.begin());
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.kp_velocity.size() ==
                this->atlasState.joint_states.kp_velocity.size(),
                "jointCommands.kp_velocity and "
                "atlasState.joint_states.kp_velocity size mismatch.");
      std::copy(this->jointCommands.kp_velocity.begin(),
                this->jointCommands.kp_velocity.end(),
                this->atlasState.joint_states.kp_velocity.begin());
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.i_effort_min.size() ==
                this->atlasState.joint_states.i_effort_min.size(),
                "jointCommands.i_effort_min and "
                "atlasState.joint_states.i_effort_min size mismatch.");
      std::copy(this->jointCommands.i_effort_min.begin(),
                this->jointCommands.i_effort_min.end(),
                this->atlasState.joint_states.i_effort_min.begin());
      // copy pid gains from jointCommands into atlasState.joint_states
      GZ_ASSERT(this->jointCommands.i_effort_max.size() ==
                this->atlasState.joint_states.i_effort_max.size(),
                "jointCommands.i_effort_max and "
                "atlasState.joint_states.i_effort_max size mismatch.");
      std::copy(this->jointCommands.i_effort_max.begin(),
                this->jointCommands.i_effort_max.end(),
                this->atlasState.joint_states.i_effort_max.begin());
    }

    // copy from atlasState.joint_states.effort into joint_states.effort
    GZ_ASSERT(this->atlasState.joint_states.effort.size() ==
              this->jointStates.effort.size(),
              "atlasState.joint_states.effort and "
              "jointStates.effort size mismatch.");
    std::copy(this->atlasState.joint_states.effort.begin(),
              this->atlasState.joint_states.effort.end(),
              this->jointStates.effort.begin());

    this->lastControllerUpdateTime = curTime;

    this->pubJointStates.publish(this->jointStates);
    this->pubAtlasState.publish(this->atlasState);

    /// controller statistics diagnostics, damages, etc.
    if (this->pubControllerStatistics.getNumSubscribers() > 0)
    {
      if ((curTime - this->lastControllerStatisticsTime).Double() >=
        1.0/this->statsUpdateRate)
      {
        atlas_msgs::ControllerStatistics msg;
        msg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
        msg.command_age = this->jointCommandsAge;
        msg.command_age_mean = this->jointCommandsAgeMean;
        msg.command_age_variance = this->jointCommandsAgeVariance /
          (this->jointCommandsAgeBuffer.size() - 1);
        msg.command_age_window_size = this->jointCommandsAgeBufferDuration;

        this->pubControllerStatistics.publish(msg);
        this->lastControllerStatisticsTime = curTime;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::OnLContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->lFootContactSensor->GetContacts();

  geometry_msgs::WrenchStamped msg;
  math::Vector3 fTotal;
  math::Vector3 tTotal;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    msg.header.stamp = ros::Time(contacts.contact(i).time().sec(),
                                 contacts.contact(i).time().nsec());
    // gzerr << "Collision between[" << contacts.contact(i).collision1()
    //           << "] and [" << contacts.contact(i).collision2() << "]\n";
    // gzerr << " t[" << this->world->GetSimTime()
    //       << "] i[" << i
    //       << "] s[" << contacts.contact(i).time().sec()
    //       << "] n[" << contacts.contact(i).time().nsec()
    //       << "] size[" << contacts.contact(i).position_size()
    //       << "]\n";

    // common::Time contactTime(contacts.contact(i).time().sec(),
    //                          contacts.contact(i).time().nsec());
    fTotal.Set(0, 0, 0);
    tTotal.Set(0, 0, 0);
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      // gzerr << j << "  Position:"
      //       << contacts.contact(i).position(j).x() << " "
      //       << contacts.contact(i).position(j).y() << " "
      //       << contacts.contact(i).position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contacts.contact(i).normal(j).x() << " "
      //       << contacts.contact(i).normal(j).y() << " "
      //       << contacts.contact(i).normal(j).z() << "\n";
      // gzerr << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_force().x(),
                            contacts.contact(i).wrench(j).body_1_force().y(),
                            contacts.contact(i).wrench(j).body_1_force().z());
      tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_torque().x(),
                            contacts.contact(i).wrench(j).body_1_torque().y(),
                            contacts.contact(i).wrench(j).body_1_torque().z());
    }
    msg.wrench.force.x = fTotal.x;
    msg.wrench.force.y = fTotal.y;
    msg.wrench.force.z = fTotal.z;
    msg.wrench.torque.x = tTotal.x;
    msg.wrench.torque.y = tTotal.y;
    msg.wrench.torque.z = tTotal.z;
    this->pubLFootContact.publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::OnRContactUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->rFootContactSensor->GetContacts();

  geometry_msgs::WrenchStamped msg;
  math::Vector3 fTotal;
  math::Vector3 tTotal;

  // GetContacts returns all contacts on the collision body
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // loop through all contact pairs to sum the total force
    // on collision1

    msg.header.stamp = ros::Time(contacts.contact(i).time().sec(),
                                 contacts.contact(i).time().nsec());

    // gzerr << "Collision between[" << contacts.contact(i).collision1()
    //           << "] and [" << contacts.contact(i).collision2() << "]\n";
    // gzerr << " t[" << this->world->GetSimTime()
    //       << "] i[" << i
    //       << "] s[" << contacts.contact(i).time().sec()
    //       << "] n[" << contacts.contact(i).time().nsec()
    //       << "] size[" << contacts.contact(i).position_size()
    //       << "]\n";

    // common::Time contactTime(contacts.contact(i).time().sec(),
    //                          contacts.contact(i).time().nsec());
    fTotal.Set(0, 0, 0);
    tTotal.Set(0, 0, 0);
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      // loop through all contacts between collision1 and collision2

      // gzerr << j << "  Position:"
      //       << contacts.contact(i).position(j).x() << " "
      //       << contacts.contact(i).position(j).y() << " "
      //       << contacts.contact(i).position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contacts.contact(i).normal(j).x() << " "
      //       << contacts.contact(i).normal(j).y() << " "
      //       << contacts.contact(i).normal(j).z() << "\n";
      // gzerr << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_force().x(),
                            contacts.contact(i).wrench(j).body_1_force().y(),
                            contacts.contact(i).wrench(j).body_1_force().z());
      tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_torque().x(),
                            contacts.contact(i).wrench(j).body_1_torque().y(),
                            contacts.contact(i).wrench(j).body_1_torque().z());
    }
    msg.wrench.force.x = fTotal.x;
    msg.wrench.force.y = fTotal.y;
    msg.wrench.force.z = fTotal.z;
    msg.wrench.torque.x = tTotal.x;
    msg.wrench.torque.y = tTotal.y;
    msg.wrench.torque.z = tTotal.z;
    this->pubRFootContact.publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::ZeroJointCommands()
{
  for (unsigned i = 0; i < this->jointCommands.name.size(); ++i)
  {
    this->jointCommands.name[i] = this->joints[i]->GetScopedName();
    this->jointCommands.position[i] = 0;
    this->jointCommands.velocity[i] = 0;
    this->jointCommands.effort[i] = 0;
    this->jointCommands.kp_position[i] = 0;
    this->jointCommands.ki_position[i] = 0;
    this->jointCommands.kd_position[i] = 0;
    this->jointCommands.kp_velocity[i] = 0;
    this->jointCommands.i_effort_min[i] = 0;
    this->jointCommands.i_effort_max[i] = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::LoadPIDGainsFromParameter()
{
  // pull down controller parameters
  for (unsigned int joint = 0; joint < this->joints.size(); ++joint)
  {
    char joint_ns[200] = "";
    snprintf(joint_ns, sizeof(joint_ns), "atlas_controller/gains/%s/",
             this->joints[joint]->GetName().c_str());
    // this is so ugly
    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
    string p_str = string(joint_ns)+"p";
    string i_str = string(joint_ns)+"i";
    string d_str = string(joint_ns)+"d";
    string i_clamp_str = string(joint_ns)+"i_clamp";
    if (!this->rosNode->getParam(p_str, p_val) ||
        !this->rosNode->getParam(i_str, i_val) ||
        !this->rosNode->getParam(d_str, d_val) ||
        !this->rosNode->getParam(i_clamp_str, i_clamp_val))
    {
      ROS_ERROR("couldn't find a param for %s", joint_ns);
      continue;
    }
    this->jointCommands.kp_position[joint]  =  p_val;
    this->jointCommands.ki_position[joint]  =  i_val;
    this->jointCommands.kd_position[joint]  =  d_val;
    this->jointCommands.i_effort_min[joint] = -i_clamp_val;
    this->jointCommands.i_effort_max[joint] =  i_clamp_val;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AtlasPlugin::SetExperimentalDampingPID(
  const atlas_msgs::Test::ConstPtr &_msg)
{
  if (_msg->damping.size() == this->joints.size())
    for (unsigned int i = 0; i < this->joints.size(); ++i)
      this->joints[i]->SetDamping(0, _msg->damping[i]);
  else
    ROS_DEBUG("joint test message contains different number of"
      " elements damping[%ld] than expected[%ld]",
      _msg->damping.size(), this->joints.size());

  boost::mutex::scoped_lock lock(this->mutex);

  if (_msg->kp_position.size() == this->jointCommands.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->jointCommands.kp_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->jointCommands.kp_position.size());

  if (_msg->ki_position.size() == this->jointCommands.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->jointCommands.ki_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->jointCommands.ki_position.size());

  if (_msg->kd_position.size() == this->jointCommands.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->jointCommands.kd_position.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->jointCommands.kd_position.size());

  if (_msg->kp_velocity.size() == this->jointCommands.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->jointCommands.kp_velocity.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->jointCommands.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->jointCommands.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->jointCommands.i_effort_min.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->jointCommands.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->jointCommands.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->jointCommands.i_effort_max.begin());
  else
    ROS_DEBUG("joint commands message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->jointCommands.i_effort_max.size());
}

void AtlasPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}
