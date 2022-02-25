from glob import glob
import os

from setuptools import setup

package_name = 'camera_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name),
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matteo Iovino',
    maintainer_email='matteo.iovino@se.abb.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_marker_pub = camera_interface.fake_marker_pub:main',
            'cube_publisher = camera_interface.cube_publisher:main',
            'object_publisher = camera_interface.maskRCNN_ros:main',
            'object_client = camera_interface.update_object_client:main',
        ],
    },
)
