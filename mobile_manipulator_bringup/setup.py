from setuptools import setup

package_name = 'mobile_manipulator_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Mobile manipulator bringup for Gazebo Harmonic with SLAM and Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'explore_and_collect = mobile_manipulator_bringup.explore_and_collect:main',
            'object_detector = mobile_manipulator_bringup.object_detector:main',
            'pick_place = mobile_manipulator_bringup.pick_place:main',
        ],
    },
)
