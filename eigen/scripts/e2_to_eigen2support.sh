o#!/bin/bash

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


    # Quaternion/AxisAngle  # Not doing bare quaternion
    sed -i -e 's/Eigen::\(Quaternion\|AxisAngle\)/Eigen::eigen2_\1/g' $f
    sed -i -e 's/\([^a-zA-Z0-9_:]\)\(Quaternion\|AxisAngle\)\([fd][^a-zA-Z0-9_]\)/\1eigen2_\2\3/g' $f

    # Transform/Translation/Scaling   # Not doing Bare instances
    sed -i -e 's/Eigen::\(Transform\|Translation\|Scaling\)/Eigen::eigen2_\1/g' $f
    # Transform/Translation/Scaling [23][fd] 
    sed -i -e 's/\([^a-zA-Z0-9_:]\)\(Transform\|Translation\|Scaling\)\([23]\?[fd]\?[^a-zA-Z0-9_]\)/\1eigen2_\2\3/g' $f

    sed -i -e 's/AlignedBox/eigen2_AlignedBox/g' $f
    sed -i -e 's/Hyperplane/eigen2_Hyperplane/g' $f
    sed -i -e 's/ParameterizedLine/eigen2_ParameterizedLine/g' $f

done
