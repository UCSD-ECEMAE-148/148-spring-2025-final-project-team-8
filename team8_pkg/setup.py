from setuptools import setup
import os
from glob import glob

package_name = 'team8_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "target_node = team8_pkg.target_node:main",
            "depth_node = team8_pkg.depth_node:main",
            "lidar_node = team8_pkg.lidar_node:main",
            'lane_detection_node = team8_pkg.lane_detection_node:main',
            'lane_guidance_node = team8_pkg.lane_guidance_node:main'
        ],
    },
)
