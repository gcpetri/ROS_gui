#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['can_bus_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/can_bus_gui']
)
setup(**d)
