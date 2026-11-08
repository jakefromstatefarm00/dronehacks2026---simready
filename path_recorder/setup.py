from glob import glob

from setuptools import setup

package_name = 'path_recorder'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='barkjacob@gmail.com',
    description='Record RC-flown paths for later replay through Sky Beta.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'recorder = path_recorder.node:main',
            'dashboard = path_recorder.dashboard:main',
            'summary = path_recorder.summary:main',
        ],
    },
)
