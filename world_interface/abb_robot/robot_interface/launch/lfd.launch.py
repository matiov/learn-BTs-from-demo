"""Launch file for the LfD framework."""

# Copyright (c) 2021 Matteo Iovino
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
from launch_ros.actions import Node
from launch_utilities.namespaces import YuMiNamespaces
from launch_utilities.process_templated_file import process_templated_file


@dataclass
class FilePaths:
    node_parameters: str = ''
    xacro: str = ''


def generate_launch_description():
    # Use machine hostname for ROS node namespaces and transformation (tf) prefixes.
    rosified_hostname = 'abb'
    namespaces = YuMiNamespaces(rosified_hostname)

    ld = LaunchDescription()

    # change config file depending on the method chosen
    config_file = 'lfd_markers.yaml'

    # Disable line buffering so messages are displayed
    os.environ['PYTHONUNBUFFERED'] = '1'

    # Process templated node parameters file (i.a. setting correct root namespace).
    interface_dir = get_package_share_directory('robot_interface')
    file_paths = FilePaths()
    try:
        file_paths.node_parameters = process_templated_file(
            os.path.join(interface_dir, 'config', config_file),
            rosified_hostname
        )
    except Exception as exception:
        print('Failed to process templated file: {0}'.format(str(exception)))
        return LaunchDescription()

    ld.add_action(
        Node(
            package='robot_interface',
            executable='lfd_gui',
            name='lfd_gui',
            namespace=namespaces.get_root_robot(),
            parameters=[file_paths.node_parameters],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            output='screen',
            emulate_tty=True
        )
    )

    return ld
