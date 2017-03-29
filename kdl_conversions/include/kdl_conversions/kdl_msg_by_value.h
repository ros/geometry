/*
 * Copyright (c) 2016, Delft Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Maarten de Vries
 */

#ifndef CONVERSIONS_KDL_MSG_BY_VALUE_H
#define CONVERSIONS_KDL_MSG_BY_VALUE_H

#include "kdl_msg.h"


namespace tf {

/// Converts a geometry_msgs Point into a KDL Vector
KDL::Vector pointMsgToKDL(const geometry_msgs::Point &in)
{
  KDL::Vector result;
  pointMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Vector into a geometry_msgs Vector3
geometry_msgs::Point pointKDLToMsg(const KDL::Vector &in)
{
  geometry_msgs::Point result;
  pointKDLToMsg(in, result);
  return result;
}

/// Converts a Pose message into a KDL Frame
KDL::Frame poseMsgToKDL(const geometry_msgs::Pose &in)
{
  KDL::Frame result;
  poseMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Frame into a Pose message
geometry_msgs::Pose poseKDLToMsg(const KDL::Frame &in)
{
  geometry_msgs::Pose result;
  poseKDLToMsg(in, result);
  return result;
}

/// Converts a quaternion message into a KDL Rotation
KDL::Rotation quaternionMsgToKDL(const geometry_msgs::Quaternion &in)
{
  KDL::Rotation result;
  quaternionMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Rotation into a message quaternion
geometry_msgs::Quaternion quaternionKDLToMsg(const KDL::Rotation &in)
{
  geometry_msgs::Quaternion result;
  quaternionKDLToMsg(in, result);
  return result;
}

/// Converts a Transform message into a KDL Frame
KDL::Frame transformMsgToKDL(const geometry_msgs::Transform &in)
{
  KDL::Frame result;
  transformMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Frame into a Transform message
geometry_msgs::Transform transformKDLToMsg(const KDL::Frame &in)
{
  geometry_msgs::Transform result;
  transformKDLToMsg(in, result);
  return result;
}

/// Converts a Twist message into a KDL Twist
KDL::Twist twistMsgToKDL(const geometry_msgs::Twist &in)
{
  KDL::Twist result;
  twistMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Twist into a Twist message
geometry_msgs::Twist twistKDLToMsg(const KDL::Twist &in)
{
  geometry_msgs::Twist result;
  twistKDLToMsg(in, result);
  return result;
}

/// Converts a Vector3 message into a KDL Vector
KDL::Vector vectorMsgToKDL(const geometry_msgs::Vector3 &in)
{
  KDL::Vector result;
  vectorMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Vector into a Vector3 message
geometry_msgs::Vector3 vectorKDLToMsg(const KDL::Vector &in)
{
  geometry_msgs::Vector3 result;
  vectorKDLToMsg(in, result);
  return result;
}

/// Converts a Wrench message into a KDL Wrench
KDL::Wrench wrenchMsgToKDL(const geometry_msgs::Wrench &in)
{
  KDL::Wrench result;
  wrenchMsgToKDL(in, result);
  return result;
}

/// Converts a KDL Wrench into a Wrench message
geometry_msgs::Wrench wrenchKDLToMsg(const KDL::Wrench &in)
{
  geometry_msgs::Wrench result;
  wrenchKDLToMsg(in, result);
  return result;
}



}


#endif



