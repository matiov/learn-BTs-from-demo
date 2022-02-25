
from glob import glob
import os

from setuptools import setup

package_name = 'robot_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
        ),
    ],
    install_requires=[
        'setuptools',
        'bt_learning',
    ],
    zip_safe=True,
    maintainer='SEMAIOV',
    maintainer_email='SEMAIOV@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lfd_gui = robot_interface.lfd_gui:main',
            'plot_clusters = robot_interface.plot_clusters:main',
            'tool_broadcaster = robot_interface.tool_broadcaster:main',
        ],
    },
)
