"""Computes the frame of the relevant cubes in the specified base frame."""

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

from typing import List

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
import quaternion
import rclpy
import rclpy.node
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


CUBES = {
    'A': [53, 54, 55, 56, 57, 58],
    'B': [20, 21, 22, 23, 24, 25],
    'C': [41, 42, 43, 44, 45, 46],
    'D': [35, 36, 37, 38, 39, 40],
    'E': [26, 27, 28, 29, 30, 31],
    'F': [47, 48, 49, 50, 51, 52]
}


class MarkerNode(rclpy.node.Node):
    """This node listens to the poses of aruco tags and publishes a mean pose to TF."""

    def __init__(self):
        global CUBES

        super().__init__('cube_publisher')

        # Declare parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('width', 0.05)
        self.declare_parameter('max_age', 2.5)
        self.declare_parameter('cubes', ['B', 'C'])

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.marker_width = self.get_parameter('width').get_parameter_value().double_value
        self.max_pose_age = self.get_parameter('max_age').get_parameter_value().double_value
        self.cube_names = self.get_parameter('cubes').get_parameter_value().string_array_value

        self.marker_sub = self.create_subscription(
                                                ArucoMarkers,
                                                '~/aruco_markers',
                                                self.marker_callback, 10
                                                )
        self.marker_info = ArucoMarkers()

        # Initialize the transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.br = TransformBroadcaster(self)

        # iterate through the cube dictionary retaining only relevant ids
        # initialize a dictionary with last poses
        self.last_poses = {}
        for name in self.cube_names:
            for marker in CUBES[name]:
                self.last_poses[marker] = None

        self.allowed_ids = []
        for id_list in CUBES.values():
            self.allowed_ids += id_list

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_object_pose)

    def marker_callback(self, msg: ArucoMarkers):
        """
        Get the recognised aruco markers in the scene.

        Args:
        ----
            msg: Aruco Marker message containing markers ids and poses.

        """
        # get the published markers
        published_markers_ids = msg.marker_ids
        # get the poses for the published markers
        now = rclpy.time.Time()
        # Delete latest poses
        for name in self.cube_names:
            for marker in CUBES[name]:
                self.last_poses[marker] = None

        for i, marker in enumerate(published_markers_ids):
            if marker not in self.allowed_ids:
                # self.get_logger().warn(f'ID {marker} not allowed.')
                continue
            target_frame = f'marker_{marker}'
            try:
                marker_tf = self.tf_buffer.lookup_transform(self.base_frame, target_frame, now)
                marker_pose = Pose()
                marker_pose.position.x = marker_tf.transform.translation.x
                marker_pose.position.y = marker_tf.transform.translation.y
                marker_pose.position.z = marker_tf.transform.translation.z
                marker_pose.orientation.x = marker_tf.transform.rotation.x
                marker_pose.orientation.y = marker_tf.transform.rotation.y
                marker_pose.orientation.z = marker_tf.transform.rotation.z
                marker_pose.orientation.w = marker_tf.transform.rotation.w
                self.last_poses[marker] = marker_pose
            except Exception:
                self.get_logger().warn(f'Frame marker_{marker} not available....')

        self.marker_info.header.stamp = msg.header.stamp
        self.marker_info.header.frame_id = msg.header.frame_id

    def publish_object_pose(self):
        """Publish the pose of the cubes by using heuristics."""
        global CUBES

        transforms = []
        for name in self.cube_names:
            # maybe do the opposite
            visible_markers = self.get_recent_marker_poses(ids=CUBES[name])
            # print('visible markers:', visible_markers)
            if len(visible_markers) == 0:
                # marker id not visible, continue
                continue

            # Position
            mean_position = self.compute_object_position(visible_markers)

            # Orientation
            orientation = self.compute_object_orientation(visible_markers)

            t = TransformStamped()
            t.header.frame_id = self.base_frame
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = name

            t.transform.translation.x = mean_position[0]
            t.transform.translation.y = mean_position[1]
            t.transform.translation.z = mean_position[2]

            t.transform.rotation.w = orientation.w
            t.transform.rotation.x = orientation.x
            t.transform.rotation.y = orientation.y
            t.transform.rotation.z = orientation.z

            self.__orientation_heuristic(t)
            transforms.append(t)

        for tr in transforms:
            self.br.sendTransform(tr)

    def compute_object_position(self, visible_markers: List[Pose]) -> np.ndarray:
        """
        Heuristic to compute the position of the cube given the visible markers and the width.

        Args
        ----
            visible_markers: markers that are visible and usable to compute the position.

        Returns
        -------
            mean_position: the position of the cube.

        """
        offset_distance = self.marker_width/2.0
        positions = np.zeros((len(visible_markers), 3))
        offset = np.array([0.0, 0.0, offset_distance])
        for i, marker in enumerate(visible_markers):
            position = np.array([marker.position.x, marker.position.y, marker.position.z])
            orientation = np.quaternion(
                                        marker.orientation.w,
                                        marker.orientation.x,
                                        marker.orientation.y,
                                        marker.orientation.z
                                        )
            orientation_inv = orientation.inverse()
            positions[i, :] = position +\
                quaternion.as_vector_part(
                    orientation_inv.conj()*quaternion.from_vector_part(-offset)*orientation_inv
                )

        mean_position = np.mean(positions, axis=0)
        return mean_position

    def compute_object_orientation(self, visible_markers: List[Pose]) -> quaternion:
        """
        Heuristic to compute the orientation of the cube given the visible markers.

        Args
        ----
            visible_markers: markers that are visible and usable to compute the orientation.

        Returns
        -------
            mean_orientation: the orientation of the cube.

        """
        marker_orientations = np.zeros((len(visible_markers),), dtype=np.quaternion)
        for i, marker in enumerate(visible_markers):
            marker_orientations[i] = np.quaternion(
                                                marker.orientation.w,
                                                marker.orientation.x,
                                                marker.orientation.y,
                                                marker.orientation.z
                                                )

        # Are we seing the top marker?
        top_visible = False
        top_orientation = None
        for marker_orientation in marker_orientations:
            # Transform z-axis to base frame
            rotation = marker_orientation.inverse()
            z_axis = quaternion.as_vector_part(
                rotation.conj()*quaternion.from_vector_part(np.array([0.0, 0.0, 1.0]))*rotation
            )
            z_axis /= np.linalg.norm(z_axis)
            z_angle = np.arccos(np.dot(z_axis, np.array([0.0, 0.0, 1.0])))
            if z_angle <= np.pi/4:
                top_visible = True
                top_orientation = marker_orientation
                break

        if top_visible:
            return top_orientation

        # Top marker is not visible. We must be seing markers on the sides.
        # Pick whatever marker to compute orientation.
        marker_orientation = marker_orientations[0]

        # Find which of x- and y-axis is most paralell to base frame z-axis.
        # I.e which has the largest dot product in magnitude.
        axes = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        # Transform axes to base frame
        rotation = marker_orientation.inverse()
        axes_base = quaternion.as_vector_part(
            rotation.conj()*quaternion.from_vector_part(axes)*rotation
        )
        dot_product = axes_base @ np.array([0.0, 0.0, 1.0]).reshape((3, 1))
        best_axis_idx = np.argmax(np.abs(dot_product))
        best_axis = axes_base[best_axis_idx]
        # Flip axis if it points in the opposite direction of base frame z-axis
        best_axis *= np.sign(dot_product[best_axis_idx])

        # The rotation axis is perpendicular to best_axis and the marker
        # z-axis.
        # Transform z-axis to base frame
        rotation = marker_orientation.inverse()
        marker_z_axis = quaternion.as_vector_part(
            rotation.conj()*quaternion.from_vector_part(np.array([0.0, 0.0, 1.0]))*rotation
        )
        # Compute the rotation axis
        rot_axis = np.cross(marker_z_axis, best_axis)

        # Transform z-axis to base frame
        rotation = marker_orientation.inverse()
        z_axis = quaternion.as_vector_part(
            rotation.conj()*quaternion.from_vector_part(np.array([0.0, 0.0, 1.0]))*rotation
        )

        mean_orientation = quaternion.from_rotation_vector(np.pi/2*rot_axis)*marker_orientation

        return mean_orientation

    def get_recent_marker_poses(self, ids: List[int]) -> List[Pose]:
        """Return poses for the ids in ids that are visible (not too old)."""
        recent = []
        for marker in ids:
            if self.last_poses[marker] is None:
                # print('skipping:', marker)
                continue
            stamp = rclpy.time.Time.from_msg(self.marker_info.header.stamp)
            age = self.get_clock().now() - stamp

            if age <= rclpy.duration.Duration(seconds=self.max_pose_age):
                recent.append(self.last_poses[marker])

        return recent

    def __orientation_heuristic(self, pose: Pose):
        """
        Restrict rotation to +- 45 degrees.

        Args:
        ----
            pose: pose of the cube.

        """
        quat = np.quaternion(
                            pose.transform.rotation.w,
                            pose.transform.rotation.x,
                            pose.transform.rotation.y,
                            pose.transform.rotation.z
                            )
        # Transform base x-axis to object frame
        base_x_axis = quaternion.as_vector_part(
            quat.conj()*quaternion.from_vector_part(np.array([1.0, 0.0, 0.0]))*quat
        )
        # Project it onto the marker xy plane
        base_x_proj = self.__proj(base_x_axis, np.array([1.0, 0.0, 0.0])) +\
            self.__proj(base_x_axis, np.array([0.0, 1.0, 0.0]))
        # Angle to x-axis
        angle = np.arctan2(base_x_proj[1], base_x_proj[0])
        # How much rotation to apply to keep angle within +-45degrees
        rotation_angle = 0
        while angle+rotation_angle > np.pi/4:
            rotation_angle -= np.pi/2
        while angle+rotation_angle < -np.pi/4:
            rotation_angle += np.pi/2

        # Apply rotation
        quat = quat*quaternion.from_rotation_vector([0.0, 0.0, -rotation_angle])

        pose.transform.rotation.w = quat.w
        pose.transform.rotation.x = quat.x
        pose.transform.rotation.y = quat.y
        pose.transform.rotation.z = quat.z

    def __proj(
        self,
        v1: np.ndarray,
        v2: np.ndarray
    ) -> np.ndarray:
        """
        Compute the projection of v1 onto v2.

        Args
        ----
            v1: vector to project.
            v2: reference vector in which the projection is computed.

        Returns
        -------
            projection: projection vector.

        """
        projection = np.dot(v1, v2)/np.linalg.norm(v2)**2 * v2
        return projection


def main(args=None):
    rclpy.init(args=args)

    marker_node = MarkerNode()
    rclpy.spin(marker_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
