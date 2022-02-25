"""Compute the homogeneous matrix given a translation and a rotation from ROS."""

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

from geometry_msgs.msg import Point, Quaternion, Vector3
import numpy as np


def homogeneous_matrix(
    position: Point or Vector3 or np.ndarray,
    orientation: Quaternion or np.ndarray,
    ros_convention: bool = True
) -> np.matrix:
    """
    Compute a homogenesous matrix given a translation and rotation.

    Args
    ----
        position: Position of the object.
        orientation: Orientation of the object.
        ros_convention: whether to use ROS msgs or numpy arrays.

    Returns
    -------
        tf_matrix: Homogeneus matrix of the given position and orientation.

    """
    tf_matrix = np.eye(4)
    if ros_convention:
        # translation
        tf_matrix[0, 3] = position.x
        tf_matrix[1, 3] = position.y
        tf_matrix[2, 3] = position.z
        # rotation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
    else:
        # translation
        tf_matrix[0, 3] = position[0]
        tf_matrix[1, 3] = position[1]
        tf_matrix[2, 3] = position[2]
        # rotation
        qx = orientation[0]
        qy = orientation[1]
        qz = orientation[2]
        qw = orientation[3]
    tf_matrix[0, 0] = 1-2*(qy**2+qz**2)
    tf_matrix[0, 1] = 2*(qx*qy-qw*qz)
    tf_matrix[0, 2] = 2*(qx*qz+qy*qw)
    tf_matrix[1, 0] = 2*(qx*qy+qw*qz)
    tf_matrix[1, 1] = 1-2*(qx**2+qz**2)
    tf_matrix[1, 2] = 2*(qy*qz-qw*qx)
    tf_matrix[2, 0] = 2*(qx*qz-qw*qy)
    tf_matrix[2, 1] = 2*(qy*qz+qw*qx)
    tf_matrix[2, 2] = 1-2*(qx**2+qy**2)

    return tf_matrix
