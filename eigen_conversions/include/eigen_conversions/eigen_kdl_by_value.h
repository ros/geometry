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

#ifndef EIGEN_KDL_CONVERSIONS_BY_VALUE_H
#define EIGEN_KDL_CONVERSIONS_BY_VALUE_H

#include "eigen_kdl.h"

namespace tf {

/// Converts a KDL rotation into an Eigen quaternion
Eigen::Quaterniond quaternionKDLToEigen(const KDL::Rotation &in)
{
  Eigen::Quaterniond result;
  quaternionKDLToEigen(in, result);
  return result;
}

/// Converts an Eigen quaternion into a KDL rotation
KDL::Rotation quaternionEigenToKDL(const Eigen::Quaterniond &in)
{
  KDL::Rotation result;
  quaternionEigenToKDL(in, result);
  return result;
}

/// Converts a KDL frame into an Eigen Isometry3d
Eigen::Isometry3d transformKDLToEigen(const KDL::Frame &in)
{
  Eigen::Isometry3d result;
  transformKDLToEigen(in, result);
  return result;
}

/// Converts an Eigen Affine3d into a KDL frame
KDL::Frame transformEigenToKDL(const Eigen::Affine3d &in)
{
  KDL::Frame result;
  transformEigenToKDL(in, result);
  return result;
}

/// Converts an Eigen Isometry3d into a KDL frame
KDL::Frame transformEigenToKDL(const Eigen::Isometry3d &in)
{
  KDL::Frame result;
  transformEigenToKDL(in, result);
  return result;
}

/// Converts a KDL twist into an Eigen matrix
Eigen::Matrix<double, 6, 1> twistKDLToEigen(const KDL::Twist &in)
{
  Eigen::Matrix<double, 6, 1> result;
  twistKDLToEigen(in, result);
  return result;
}

/// Converts an Eigen matrix into a KDL Twist
KDL::Twist twistEigenToKDL(const Eigen::Matrix<double, 6, 1> &in)
{
  KDL::Twist result;
  twistEigenToKDL(in, result);
  return result;
}

/// Converts a KDL vector into an Eigen matrix
Eigen::Matrix<double, 3, 1> vectorKDLToEigen(const KDL::Vector &in)
{
  Eigen::Matrix<double, 3, 1> result;
  vectorKDLToEigen(in, result);
  return result;
}

/// Converts an Eigen matrix into a KDL vector
KDL::Vector vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> &in)
{
  KDL::Vector result;
  vectorEigenToKDL(in, result);
  return result;
}

/// Converts a KDL wrench into an Eigen matrix
Eigen::Matrix<double, 6, 1> wrenchKDLToEigen(const KDL::Wrench &in)
{
  Eigen::Matrix<double, 6, 1> result;
  wrenchKDLToEigen(in, result);
  return result;
}

/// Converts an Eigen matrix into a KDL wrench
KDL::Wrench wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> &in)
{
  KDL::Wrench result;
  wrenchEigenToKDL(in, result);
  return result;
}


} // namespace

#endif
