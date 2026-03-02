from setuptools import setup
import os
from glob import glob

package_name = 'ros_rover'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='steve@rpi4.local',
    description='Hardware bridge for VIAM Rover 1 on Raspberry Pi 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viam_driver = ros_rover.viam_driver:main'
        ],
    },
)
