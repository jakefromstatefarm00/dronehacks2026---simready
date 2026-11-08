"""dronehacks_pathplan: the whole pathfinding folder is one ROS 2 (ament_python) package.

Installs three Python packages -- dronehacks_pathplan (the ROS node, mission, launch helpers),
pathplan (the planner) and drl (the course loader) -- plus the launch files and the published
course (config/ + course/) into share/dronehacks_pathplan, so the node finds the map with or
without --symlink-install. gzsim/ and tests/ are development tools and are not installed.
"""
from glob import glob

from setuptools import setup

package_name = 'dronehacks_pathplan'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 'pathplan', 'drl'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/course', glob('course/*.obj') + glob('course/*.mtl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jolie',
    maintainer_email='jolieznli@gmail.com',
    description='Sky Beta competitor node flying the course with an online path planner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pathplan_node = dronehacks_pathplan.node:main',
            'sim_pilot = dronehacks_pathplan.sim_pilot:main',
            'fake_px4 = dronehacks_pathplan.fake_px4:main',
            'check_pipeline = dronehacks_pathplan.check_pipeline:main',
            'report = dronehacks_pathplan.report:main',
            'dry_run = dronehacks_pathplan.dry_run:main',
        ],
    },
)
