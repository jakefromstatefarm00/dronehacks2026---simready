from glob import glob
from setuptools import setup

package_name = 'dronehacks_pathplan'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jolie',
    maintainer_email='jolieznli@gmail.com',
    description='Skybeta competitor node flying the course with an online path planner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pathplan_mission = dronehacks_pathplan.pathplan_mission:main',
        ],
    },
)
