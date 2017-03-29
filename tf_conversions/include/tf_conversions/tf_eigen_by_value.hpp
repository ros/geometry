/*
 * Copyright (c) 2016, Delft Robotics.
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

// Author: Maarten de Vries, Ronald Ensing

#ifndef CONVERSIONS_TF_EIGEN_BY_VALUE_H
#define CONVERSIONS_TF_EIGEN_BY_VALUE_H

#include "tf/transform_datatypes.h"
#include "tf_eigen.h"

namespace tf {

/// Converts a tf Matrix3x3 into an Eigen Quaternion
Eigen::Matrix3d matrixTFToEigen(const tf::Matrix3x3 &in) {
    Eigen::Matrix3d result;
    matrixTFToEigen(in, result);
    return result;
}

/// Converts an Eigen Quaternion into a tf Matrix3x3
tf::Matrix3x3 matrixEigenToTF(const Eigen::Matrix3d &in) {

    tf::Matrix3x3 result;
    matrixEigenToTF(in, result);
    return result;
}

/// Converts a tf Pose into an Eigen Isometry3d
Eigen::Isometry3d poseTFToEigen(const tf::Pose &in) {

    Eigen::Isometry3d result;
    poseTFToEigen(in, result);
    return result;
}

/// Converts an Eigen Affine3d into a tf Transform
tf::Pose poseEigenToTF(const Eigen::Affine3d &in) {

    tf::Pose result;
    poseEigenToTF(in, result);
    return result;
}

/// Converts an Eigen Isometry3d into a tf Transform
tf::Pose poseEigenToTF(const Eigen::Isometry3d &in) {

    tf::Pose result;
    poseEigenToTF(in, result);
    return result;
}

/// Converts a tf Quaternion into an Eigen Quaternion
Eigen::Quaterniond quaternionTFToEigen(const tf::Quaternion &in) {
    Eigen::Quaterniond result;
    quaternionTFToEigen(in, result);
    return result;
}

/// Converts an Eigen Quaternion into a tf Quaternion
tf::Quaternion quaternionEigenToTF(const Eigen::Quaterniond &in) {
    tf::Quaternion result;
    quaternionEigenToTF(in, result);
    return result;
}

/// Converts a tf Transform into an Eigen Isometry3d
Eigen::Isometry3d transformTFToEigen(const tf::Transform &in) {

    Eigen::Isometry3d result;
    transformTFToEigen(in, result);
    return result;
}

/// Converts an Eigen Affine3d into a tf Transform
tf::Transform transformEigenToTF(const Eigen::Affine3d &in) {

    tf::Transform result;
    transformEigenToTF(in, result);
    return result;
}

/// Converts an Eigen Isometry3d into a tf Transform
tf::Transform transformEigenToTF(const Eigen::Isometry3d &in) {

    tf::Transform result;
    transformEigenToTF(in, result);
    return result;
}

/// Converts a tf Vector3 into an Eigen Vector3d
Eigen::Vector3d vectorTFToEigen(const tf::Vector3 &in) {
    Eigen::Vector3d result;
    vectorTFToEigen(in, result);
    return result;
}

/// Converts an Eigen Vector3d into a tf Vector3
tf::Vector3 vectorEigenToTF(const Eigen::Vector3d &in) {
    tf::Vector3 result;
    vectorEigenToTF(in, result);
    return result;
}

} // namespace tf

#endif
