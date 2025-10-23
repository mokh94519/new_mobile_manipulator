from setuptools import setup
import os
from glob import glob

package_name = 'mobile_manipulator_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Mobile manipulator simulation with Gazebo Harmonic, SLAM Toolbox, Nav2, and object collection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore_and_collect = mobile_manipulator_bringup.explore_and_collect:main',
            'arm_controller = mobile_manipulator_bringup.arm_controller:main',
            'object_detector = mobile_manipulator_bringup.object_detector:main',
        ],
    },
)
