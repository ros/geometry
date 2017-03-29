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

#ifndef EIGEN_MSG_CONVERSIONS_BY_VALUE_H
#define EIGEN_MSG_CONVERSIONS_BY_VALUE_H

#include "eigen_msg.h"

namespace tf {

/// Converts a Point message into an Eigen Vector
Eigen::Vector3d pointMsgToEigen(const geometry_msgs::Point &in)
{
  Eigen::Vector3d result;
  pointMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen Vector into a Point message
geometry_msgs::Point pointEigenToMsg(const Eigen::Vector3d &in)
{
  geometry_msgs::Point result;
  pointEigenToMsg(in, result);
  return result;
}

/// Converts a Pose message into an Eigen Isometry3d
Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose &in)
{
  Eigen::Isometry3d result;
  poseMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen Affine3d into a Pose message
geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &in)
{
  geometry_msgs::Pose result;
  poseEigenToMsg(in, result);
  return result;
}

/// Converts an Eigen Isometry3d into a Pose message
geometry_msgs::Pose poseEigenToMsg(const Eigen::Isometry3d &in)
{
  geometry_msgs::Pose result;
  poseEigenToMsg(in, result);
  return result;
}

/// Converts a Quaternion message into an Eigen Quaternion
Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion &in)
{
  Eigen::Quaterniond result;
  quaternionMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen Quaternion into a Quaternion message
geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond &in)
{
  geometry_msgs::Quaternion result;
  quaternionEigenToMsg(in, result);
  return result;
}

/// Converts a Transform message into an Eigen Isometry3d
Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::Transform &in)
{
  Eigen::Isometry3d result;
  transformMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen Affine3d into a Transform message
geometry_msgs::Transform transformEigenToMsg(const Eigen::Affine3d &in)
{
  geometry_msgs::Transform result;
  transformEigenToMsg(in, result);
  return result;
}

/// Converts an Eigen Isometry3d into a Transform message
geometry_msgs::Transform transformEigenToMsg(const Eigen::Isometry3d &in)
{
  geometry_msgs::Transform result;
  transformEigenToMsg(in, result);
  return result;
}

/// Converts a Twist message into an Eigen matrix
Eigen::Matrix<double,6,1> twistMsgToEigen(const geometry_msgs::Twist &in)
{
  Eigen::Matrix<double,6,1> result;
  twistMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen matrix into a Twist message
geometry_msgs::Twist twistEigenToMsg(const Eigen::Matrix<double,6,1> &in)
{
  geometry_msgs::Twist result;
  twistEigenToMsg(in, result);
  return result;
}

/// Converts a Vector message into an Eigen Vector
Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &in)
{
  Eigen::Vector3d result;
  vectorMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen Vector into a Vector message
geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &in)
{
  geometry_msgs::Vector3 result;
  vectorEigenToMsg(in, result);
  return result;
}

/// Converts a Wrench message into an Eigen matrix
Eigen::Matrix<double,6,1> wrenchMsgToEigen(const geometry_msgs::Wrench &in)
{
  Eigen::Matrix<double,6,1> result;
  wrenchMsgToEigen(in, result);
  return result;
}

/// Converts an Eigen matrix into a Wrench message
geometry_msgs::Wrench wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &in)
{
  geometry_msgs::Wrench result;
  wrenchEigenToMsg(in, result);
  return result;
}

/// Converts an Eigen matrix into a Float64MultiArray message
template <class Derived>
std_msgs::Float64MultiArray matrixEigenToMsg(const Eigen::MatrixBase<Derived> &in)
{
  std_msgs::Float64MultiArray result;
  matrixEigenToMsg<Derived>(in, result);
  return result;
}

} // namespace

#endif
