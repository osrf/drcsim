/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "apps/atlasRobot/Humanoid.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
//#include "dart/constraint/OldConstraintDynamics.h"
#include "dart/collision/CollisionDetector.h"

#include "apps/atlasRobot/State.h"

// Macro for functions not implemented yet
#define NOT_YET(FUNCTION) std::cout << #FUNCTION\
                                  << "Not implemented yet."\
                                  << std::endl;

using namespace std;

using namespace Eigen;

using namespace dart::dynamics;
using namespace dart::constraint;

//==============================================================================
Humanoid::Humanoid(Skeleton* _skeleton)
  : mSkeleton(NULL),
    mPelvis(NULL),
    mLeftThigh(NULL),
    mRightThigh(NULL),
    mLeftFoot(NULL),
    mRightFoot(NULL)
{

}

//==============================================================================
Humanoid::~Humanoid()
{

}

//==============================================================================
Skeleton* Humanoid::getSkeleton()
{
  return mSkeleton;
}

//==============================================================================
BodyNode* Humanoid::getPelvis()
{
  return mPelvis;
}

//==============================================================================
BodyNode* Humanoid::getLeftThigh()
{
  return mLeftThigh;
}

//==============================================================================
BodyNode* Humanoid::getRightThigh()
{
  return mRightThigh;
}

//==============================================================================
BodyNode* Humanoid::getLeftFoot()
{
  return mLeftFoot;
}

//==============================================================================
BodyNode* Humanoid::getRightFoot()
{
  return mRightFoot;
}

//==============================================================================
AtlasRobot::AtlasRobot(Skeleton* _skeleton)
  : Humanoid(_skeleton)
{

}

//==============================================================================
AtlasRobot::~AtlasRobot()
{
  mPelvis     = mSkeleton->getBodyNode("pelvis");
  mLeftFoot   = mSkeleton->getBodyNode("l_foot");
  mRightFoot  = mSkeleton->getBodyNode("r_foot");
  mLeftThigh  = mSkeleton->getBodyNode("l_uleg");
  mRightThigh = mSkeleton->getBodyNode("r_uleg");

  assert(mPelvis     != NULL);
  assert(mLeftFoot   != NULL);
  assert(mRightFoot  != NULL);
  assert(mLeftThigh  != NULL);
  assert(mRightThigh != NULL);
}
