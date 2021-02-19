#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['my_gui_pkg'],
    package_dir={'': 'src'},
    scripts=['scripts/my_gui_pkg']
)

setup(**d)
