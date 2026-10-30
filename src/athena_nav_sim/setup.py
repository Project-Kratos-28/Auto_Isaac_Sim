from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'athena_nav_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarsh',
    maintainer_email='aarshbhatt2007@gmail.com',
    description='Kratos autonomy stack (GLIM, pcd2pgm, Nav2) on the Isaac Sim digital twin',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'rover_bridge = athena_nav_sim.rover_bridge:main',
            'glim_map_odom = athena_nav_sim.glim_map_odom:main',
            'send_waypoints = athena_nav_sim.send_waypoints:main',
            'map_seed = athena_nav_sim.map_seed:main',
        ],
    },
)
