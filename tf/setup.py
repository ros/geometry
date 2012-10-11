#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['tf']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/bullet_migration_sed.py',
                'scripts/tf_remap',
                'scripts/view_frames']
d['install_requires'] = ['genmsg', 'genpy', 'roslib', 'rospkg', 'geometry_msgs', 'sensor_msgs', 'std_msgs']

setup(**d)
