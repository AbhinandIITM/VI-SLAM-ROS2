from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinand',
    maintainer_email='your_email@example.com',
    description='AprilTag SLAM with IMU and camera',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Note the dot notation: package.module:function
            'torso_estimator = apriltag_slam.torso_estimator_node:main',
            'torso_stabilizer = apriltag_slam.torso_stabilizer_node:main',
            'random_tag_mover = apriltag_slam.random_tag_mover:main',
        ],
    },
)