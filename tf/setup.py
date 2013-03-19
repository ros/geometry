#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf'],
    package_dir={'': 'src'},
    scripts=['scripts/bullet_migration_sed.py',
             'scripts/tf_remap',
             'scripts/view_frames'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'geometry_msgs', 'sensor_msgs', 'std_msgs']
)

setup(**d)
