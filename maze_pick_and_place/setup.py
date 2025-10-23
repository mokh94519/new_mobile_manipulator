from setuptools import setup

package_name = 'maze_pick_and_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/maze_sim.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/maze_world.sdf']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Maze world pick-and-place with Gazebo Harmonic and ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place_node = maze_pick_and_place.pick_and_place_node:main',
        ],
    },
)
