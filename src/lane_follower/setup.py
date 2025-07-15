from glob import glob
from setuptools import setup
import os

package_name = 'lane_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Lane follower node using OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follower = lane_follower.lane_follower:main',
            'lane_driver = lane_follower.lane_driver:main',
            'twist_filter_node = lane_follower.twist_filter_node:main',
            'lane_control_pid = lane_follower.lane_control_pid:main',
        ],
    },
)
