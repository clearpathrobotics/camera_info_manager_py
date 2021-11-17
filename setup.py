#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['camera_info_manager'],
    package_dir={'': 'src'},
    install_requires=['rospkg', 'yaml'],
    )

setup(**d)
