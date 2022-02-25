# ==================================================================================================
#
# Copyright 2021 ABB
#
# ==================================================================================================

from dataclasses import dataclass

from abb_myumi_launch_utilities import NamespacesMYuMiV2
from launch import LaunchDescription
from launch_ros.actions import Node


@dataclass
class FilePaths:
    node_parameters: str = ''
    xacro: str = ''


def generate_launch_description():
    # Use machine hostname for ROS node namespaces and transformation (tf) prefixes.
    rosified_hostname = 'myumi_003'
    namespaces = NamespacesMYuMiV2(rosified_hostname)
    tf_prefix = rosified_hostname + '_'

    ld = LaunchDescription()

    # RViz configuration file.
    """
    rviz_configuration_file_path = os.path.join(
        get_package_share_directory('abb_yumi_description'),
        'rviz',
        'myumi_003.rviz'
    )
    # Visualization
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-f', tf_prefix + 'map', '-d', rviz_configuration_file_path]
        )
    )
    """

    sensor_camera_ns = namespaces.get_root_sensors() + '/' + namespaces.camera
    # NB: rectification is done in the mobile manipulator bringup to overcome delays in image transmission
    # Rectify images
    ld.add_action(
        # Object shape + pose estimation
        Node(
            package='sdfest_ros',
            executable='sdfest_node',
            name='sdfest_node',
            namespace=sensor_camera_ns,
            remappings=[
                ('camera_info', 'depth/camera_info'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
                ('~/get_objects', 'get_objects'),
                ('~/get_table', 'get_table')
            ],
            parameters=[{
                'visualize_optimization': True,
                'visualize_mask_rcnn_predictions': True,
                'confidence_threshold': 0.03
            }],
            output='screen',
            emulate_tty=True,
        )
    )

    # Grasping service
    ld.add_action(
        Node(
            package='grasping',
            executable='grasping_service',
            name='compute_grasping_point',
            namespace=namespaces.get_root_robot(),
            remappings=[
                ('~/compute_grasping', 'compute_grasping'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            parameters=[{
                'base_frame': tf_prefix + 'yumi_base_link',
                'mug_offsetZ': 0.02,
                'knife_offsetZ': -0.01,
                'knife_offsetX': -0.02,
            }],
            output='screen',
        )
    )

    return ld
