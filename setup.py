# setup.py
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['drone_slam_simulation'],
    package_dir={'': 'src'}
)

setup(**d)

