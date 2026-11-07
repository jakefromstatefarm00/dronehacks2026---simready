from glob import glob

from setuptools import setup

package_name = 'team_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.rl', package_name + '.sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/checkpoints', glob('config/checkpoints/*.yaml')),
        ('share/' + package_name + '/policies', glob('policies/*.npz') + glob('policies/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Jacob',
    maintainer_email='barkjacob@gmail.com',
    description='Checkpoint mission with a trainable policy, replacing square_mission.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_node = team_mission.mission_node:main',
            'sim_pilot = team_mission.sim.sim_pilot:main',
            'sim_odom_adapter = team_mission.sim.sim_odom_adapter:main',
        ],
    },
)
