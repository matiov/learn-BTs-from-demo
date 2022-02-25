"""
Launch file for the cube detection.

This script spawns the YuMi robot and the Azure Kinect camera drivers.
It runs the Aruco marker detection, with the heuristic to publish a frame in the cubes center.
It creates a frame corresponding to the Tool set in the RAPID controller.

"""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from dataclasses import dataclass
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_utilities.namespaces import YuMiNamespaces
from launch_utilities.process_templated_file import process_templated_file


@dataclass
class FilePaths:

    node_parameters: str = ''
    xacro: str = ''


def yumi_bringup(launch_description, rosified_hostname, namespaces, platform_version, tf_prefix):
    """
    Generate launch description for a second generation mobile YuMi platform.

    Intended for bringing up core ROS system.

    Returns
    -------
        launch description to the ROS launch system.

    """
    # ==============================================================================================
    # Preparations
    # ==============================================================================================
    # Path to this package's share directory.
    this_package_share_directory = get_package_share_directory('abb_yumi_bringup')

    ld = launch_description

    # ----------------------------------------------------------------------------------------------
    # Environment variables
    # ----------------------------------------------------------------------------------------------
    # Set logging output format.
    #
    # Note: Intended for debugging and testing.
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

    # Set colorized output for non-Windows systems (i.e. OS name is not 'nt').
    #
    # Note: Intended for debugging and testing.
    if os.name != 'nt':
        os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    # ----------------------------------------------------------------------------------------------
    # File paths
    # ----------------------------------------------------------------------------------------------
    file_paths = FilePaths()

    # Process templated node parameters file (i.a. setting correct root namespace).
    try:
        file_paths.node_parameters = process_templated_file(
            os.path.join(
                this_package_share_directory,
                'config',
                platform_version,
                'core_template.yaml'
            ),
            rosified_hostname
        )
    except Exception as exception:
        print('Failed to process templated file: {0}'.format(str(exception)))
        return LaunchDescription()

    # File path to the xacro file (i.e. used to generate the URDF of the hardware).
    file_paths.xacro = os.path.join(
        get_package_share_directory('abb_yumi_description'),
        'urdf',
        platform_version + '.xacro'
    )

    # Command substitution for generating the URDF (i.e. when the launch description is processed).
    robot_description_substitution = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', file_paths.xacro,
        ' ', 'prefix:=', tf_prefix
    ])

    # ----------------------------------------------------------------------------------------------
    # Hardware description
    # ----------------------------------------------------------------------------------------------
    # Transform publisher node (i.e. for tansforms specified in an URDF).
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespaces.get_root(),
            name='robot_state_publisher_node',
            parameters=[{'robot_description': robot_description_substitution}],
            remappings=[('joint_states', 'merged_joint_states')]
        )
    )

    # Joint state publisher node (i.e. for merging platform and robot joint states).
    ld.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespaces.get_root(),
            name='joint_state_publisher_node',
            parameters=[file_paths.node_parameters],
            remappings=[('joint_states', 'merged_joint_states')]
        )
    )

    # ----------------------------------------------------------------------------------------------
    # Peripherals (e.g. sensors and joystick)
    # ----------------------------------------------------------------------------------------------

    # Sensor related nodes.
    ld.add_action(
        GroupAction([
            PushRosNamespace(namespace=namespaces.get_root_sensors()),
            Node(
                package='azure_kinect_ros_driver',
                executable='node',
                namespace=namespaces.camera,
                parameters=[file_paths.node_parameters]
            ),
        ])
    )

    # ----------------------------------------------------------------------------------------------
    #  ABB robot
    # ----------------------------------------------------------------------------------------------
    # ABB robot related nodes.
    ld.add_action(
        GroupAction([
            PushRosNamespace(namespace=namespaces.get_root_robot()),

            # RWS state publisher node (e.g. for gathering and publishing system states).
            Node(
                package='abb_rws_state_publisher',
                executable='rws_state_publisher_node',
                name='rws_state_publisher_node',
                parameters=[file_paths.node_parameters],
                remappings=[
                    ('~/joint_states', 'joint_states'),
                    ('~/system_states', 'rws/system_states'),
                    ('~/sm_addin/runtime_states', 'rws/sm_addin/runtime_states')
                ]
            ),
            # RWS service provider node (e.g. for requesting start/stop of RAPID execution).
            Node(
                package='abb_rws_service_provider',
                executable='rws_service_provider_node',
                name='rws_service_provider_node',
                parameters=[file_paths.node_parameters],
                remappings=[
                    ('~/system_states', 'rws/system_states'),
                    ('~/sm_addin/runtime_states', 'rws/sm_addin/runtime_states'),
                    ('~/get_file_contents', 'rws/get_file_contents'),
                    ('~/get_io_signal', 'rws/get_io_signal'),
                    ('~/get_mechunit_robtarget', 'rws/get_mechunit_robtarget'),
                    ('~/get_rapid_bool', 'rws/get_rapid_bool'),
                    ('~/get_rapid_dnum', 'rws/get_rapid_dnum'),
                    ('~/get_rapid_num', 'rws/get_rapid_num'),
                    ('~/get_rapid_string', 'rws/get_rapid_string'),
                    ('~/get_rapid_symbol', 'rws/get_rapid_symbol'),
                    ('~/get_speed_ratio', 'rws/get_speed_ratio'),
                    ('~/pp_to_main', 'rws/pp_to_main'),
                    ('~/set_file_contents', 'rws/set_file_contents'),
                    ('~/set_io_signal', 'rws/set_io_signal'),
                    ('~/set_rapid_bool', 'rws/set_rapid_bool'),
                    ('~/set_rapid_dnum', 'rws/set_rapid_dnum'),
                    ('~/set_rapid_num', 'rws/set_rapid_num'),
                    ('~/set_rapid_string', 'rws/set_rapid_string'),
                    ('~/set_rapid_symbol', 'rws/set_rapid_symbol'),
                    ('~/set_speed_ratio', 'rws/set_speed_ratio'),
                    ('~/sm_addin/get_egm_settings', 'rws/sm_addin/get_egm_settings'),
                    ('~/sm_addin/run_rapid_routine', 'rws/sm_addin/run_rapid_routine'),
                    ('~/sm_addin/run_sg_routine', 'rws/sm_addin/run_sg_routine'),
                    ('~/sm_addin/set_egm_settings', 'rws/sm_addin/set_egm_settings'),
                    ('~/sm_addin/set_rapid_routine', 'rws/sm_addin/set_rapid_routine'),
                    ('~/sm_addin/set_sg_command', 'rws/sm_addin/set_sg_command'),
                    ('~/sm_addin/start_egm_joint', 'rws/sm_addin/start_egm_joint'),
                    ('~/sm_addin/start_egm_pose', 'rws/sm_addin/start_egm_pose'),
                    ('~/sm_addin/start_egm_stream', 'rws/sm_addin/start_egm_stream'),
                    ('~/sm_addin/stop_egm', 'rws/sm_addin/stop_egm'),
                    ('~/sm_addin/stop_egm_stream', 'rws/sm_addin/stop_egm_stream'),
                    ('~/start_rapid', 'rws/start_rapid'),
                    ('~/stop_rapid', 'rws/stop_rapid'),
                    ('~/turn_motors_off', 'rws/turn_motors_off'),
                    ('~/turn_motors_on', 'rws/turn_motors_on')
                ]
            ),
            # RAPID routine caller node (i.e. for requesting predefined RAPID routines to be run).
            Node(
                package='abb_rapid_routine_caller',
                executable='rapid_routine_caller_node',
                name='rapid_routine_caller_node',
                remappings=[
                    ('~/system_states', 'rws/system_states'),
                    ('~/sm_addin/runtime_states', 'rws/sm_addin/runtime_states'),
                    ('~/get_rapid_string', 'rws/get_rapid_string'),
                    ('~/pp_to_main', 'rws/pp_to_main'),
                    ('~/start_rapid', 'rws/start_rapid'),
                    ('~/stop_rapid', 'rws/stop_rapid'),
                    ('~/sm_addin/set_rapid_routine', 'rws/sm_addin/set_rapid_routine'),
                    ('~/sm_addin/run_rapid_routine', 'rws/sm_addin/run_rapid_routine')
                ]
            ),

        ])
    )

    # RViz configuration file.
    rviz_configuration_file_path = os.path.join(
        get_package_share_directory('camera_interface'),
        'rviz',
        'yumi_marker.rviz'
    )

    # Visualization
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=[
                '-f', tf_prefix + platform_version + '_base_link',
                '-d', rviz_configuration_file_path
            ]
        )
    )

    return ld


def generate_launch_description():
    # Use hostname for ROS node namespaces and transformation (tf) prefixes.
    # Platform version.
    platform_version = 'yumi'

    # Use machine hostname for ROS node namespaces and transformation (tf) prefixes.
    # NOTE: the Rviz config file is saved with topic names depending on this parameter!
    rosified_hostname = 'abb'
    namespaces = YuMiNamespaces(rosified_hostname)
    tf_prefix = ''
    if rosified_hostname != '':
        tf_prefix = rosified_hostname + '_'

    ld = LaunchDescription()
    ld = yumi_bringup(ld, rosified_hostname, namespaces, platform_version, tf_prefix)

    # Disable line buffering so messages are displayed
    os.environ['PYTHONUNBUFFERED'] = '1'

    # Process templated node parameters file (i.a. setting correct root namespace).
    # Transform publisher for the Calibration Tool
    camera_dir = get_package_share_directory('aruco_detection')
    file_paths = FilePaths()
    try:
        file_paths.node_parameters = process_templated_file(
            os.path.join(camera_dir, 'config', 'marker_detection.yaml'),
            rosified_hostname
        )
    except Exception as exception:
        print('Failed to process templated file: {0}'.format(str(exception)))
        return LaunchDescription()

    sensor_camera_ns = namespaces.get_root_camera()
    # Rectify images
    ld.add_action(
        Node(
            package='image_proc',
            namespace=sensor_camera_ns + '/rgb',
            executable='image_proc',
            name='d2c_image_proc',
            remappings=[('image', 'image_raw')]
        )
    )
    ld.add_action(
        Node(
            package='image_proc',
            namespace=sensor_camera_ns + '/depth',
            executable='image_proc',
            name='d2c_image_proc',
            remappings=[('image', 'image_raw')]
        )
    )
    ld.add_action(
        Node(
            package='image_proc',
            namespace=sensor_camera_ns + '/rgb_to_depth',
            executable='image_proc',
            name='d2c_image_proc',
            remappings=[('image', 'image_raw')]
        )
    )

    ld.add_action(
        Node(
            package='aruco_detection',
            executable='aruco_node',
            name='aruco_node',
            namespace=sensor_camera_ns,
            parameters=[file_paths.node_parameters],
            remappings=[
                ('~/camera/camera_info', 'rgb/camera_info'),
                ('~/camera/image_raw', 'rgb/image_raw'),
                ('~/aruco_poses', 'aruco_poses'),
                ('~/aruco_markers', 'aruco_markers'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
    ),

    ld.add_action(
        Node(
            package='aruco_detection',
            executable='marker_publisher',
            name='marker_publisher',
            namespace=sensor_camera_ns,
            parameters=[file_paths.node_parameters],
            remappings=[
                ('~/aruco_markers', 'aruco_markers'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
    ),

    # Process templated node parameters file (i.a. setting correct root namespace).
    # Transform publisher for the Task Tool
    camera_dir = get_package_share_directory('camera_interface')
    file_paths = FilePaths()
    try:
        file_paths.node_parameters = process_templated_file(
            os.path.join(camera_dir, 'config', 'cube_detection.yaml'),
            rosified_hostname
        )
    except Exception as exception:
        print('Failed to process templated file: {0}'.format(str(exception)))
        return LaunchDescription()

    ld.add_action(
        Node(
            package='robot_interface',
            executable='tool_broadcaster',
            name='tool_broadcaster_node',
            output='screen',
            namespace=namespaces.get_root_robot(),
            parameters=[file_paths.node_parameters],
            remappings=[('~/get_mechunit_robtarget', 'rws/get_mechunit_robtarget')]
        ),
    )

    ld.add_action(
        Node(
            package='camera_interface',
            executable='cube_publisher',
            name='cube_publisher',
            output='screen',
            namespace=sensor_camera_ns,
            parameters=[file_paths.node_parameters],
            remappings=[
                ('~/aruco_markers', 'aruco_markers'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            emulate_tty=True
        ),
    ),

    return ld
