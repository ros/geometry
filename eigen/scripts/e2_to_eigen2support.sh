#!/bin/bash

if [ "$1" == "" ]; then
    echo "Usage: $0 <path>"
    echo ""
    echo "Prefixes Eigen2 specific methods with eigen2_ recursively in all files in <path>."
    exit 1
fi

for f in $(grep -lr 'namespace Eigen\|Eigen::\|Eigen/' $1 | grep -v '.svn'); do
    echo "Processing $f"

    sed -i -e 's/ei_toRotationMatrix/eigen2_ei_toRotationMatrix/g' $f
    sed -i -e 's/ei_quaternion_assign_impl/eigen2_ei_quaternion_assign_impl/g' $f
    sed -i -e 's/ei_transform_product_impl/eigen2_ei_transform_product_impl/g' $f


    sed -i -e 's/RotationBase/eigen2_RotationBase/g' $f


    sed -i -e 's/Rotation2D/eigen2_Rotation2D/g' $f
    sed -i -e 's/Rotation2Df/eigen2_Rotation2Df/g' $f
    sed -i -e 's/Rotation2Dd/eigen2_Rotation2Dd/g' $f


    sed -i -e 's/Eigen::Quaternion/Eigen::eigen2_Quaternion/g' $f
    sed -i -e 's/Quaternionf/eigen2_Quaternionf/g' $f
    sed -i -e 's/Quaterniond/eigen2_Quaterniond/g' $f

    sed -i -e 's/Eigen::AxisAngle/Eigen::eigen2_AxisAngle/g' $f
    sed -i -e 's/AxisAnglef/eigen2_AxisAnglef/g' $f
    sed -i -e 's/AxisAngled/eigen2_AxisAngled/g' $f

    sed -i -e 's/Eigen::Transform/Eigen::eigen2_Transform/g' $f
    sed -i -e 's/Transform2f/eigen2_Transform2f/g' $f
    sed -i -e 's/Transform2d/eigen2_Transform2d/g' $f
    sed -i -e 's/Transform3f/eigen2_Transform3f/g' $f
    sed -i -e 's/Transform3d/eigen2_Transform3d/g' $f

    sed -i -e 's/Eigen::Translation/Eigen::eigen2_Translation/g' $f
    sed -i -e 's/Translation2f/eigen2_Translation2f/g' $f
    sed -i -e 's/Translation2d/eigen2_Translation2d/g' $f
    sed -i -e 's/Translation3f/eigen2_Translation3f/g' $f
    sed -i -e 's/Translation3d/eigen2_Translation3d/g' $f

    sed -i -e 's/Eigen::Scaling/Eigen::eigen2_Scaling/g' $f
    sed -i -e 's/Scaling2f/eigen2_Scaling2f/g' $f
    sed -i -e 's/Scaling2d/eigen2_Scaling2d/g' $f
    sed -i -e 's/Scaling3f/eigen2_Scaling3f/g' $f
    sed -i -e 's/Scaling3d/eigen2_Scaling3d/g' $f

    sed -i -e 's/AlignedBox/eigen2_AlignedBox/g' $f
    sed -i -e 's/Hyperplane/eigen2_Hyperplane/g' $f
    sed -i -e 's/ParameterizedLine/eigen2_ParameterizedLine/g' $f

done
