from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agc_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config', 'calibration'),
            glob('config/calibration/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chea Vitou',
    maintainer_email='6023010001@camtech.edu.kh',
    description='SLAM module for Air-Ground Collaborative Mapping.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_interface = agc_slam.slam_interface:main',

            'check_frames = agc_slam.frame_check:main',
        ],
    },
)
