"""This node retrieves the marker poses from the aruco topic and publishes them to TF."""

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

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped
import numpy as np
from perception_utils import transformations
from perception_utils.homogeneous_matrix import homogeneous_matrix
import rclpy
import rclpy.node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


class MarkerPub(rclpy.node.Node):

    def __init__(self):
        super().__init__('marker_publisher')

        # Declare and read parameters
        self.declare_parameter('reference_frame', '')

        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        # Initialize the transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(ArucoMarkers, '~/aruco_markers', self.tf_broadcaster, 1)


    def tf_broadcaster(self, msg: ArucoMarkers):
        """
        Read the marker topic and publish marker pose to /tf.

        Args:
        ----
            msg: The callback message.

        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.reference_frame
        if self.reference_frame == '':
            # no need to transform the marker pose to another frame
            t.header.frame_id = msg.header.frame_id

        for i, id in enumerate(msg.marker_ids):
            if id == 0.0:
                continue
            else:
                t.child_frame_id = f'marker_{id}'
                if self.reference_frame == '':
                    t.transform.translation.x = msg.poses[i].position.x
                    t.transform.translation.y = msg.poses[i].position.y
                    t.transform.translation.z = msg.poses[i].position.z
                    t.transform.rotation.x = msg.poses[i].orientation.x
                    t.transform.rotation.y = msg.poses[i].orientation.y
                    t.transform.rotation.z = msg.poses[i].orientation.z
                    t.transform.rotation.w = msg.poses[i].orientation.w
                else:
                    t.transform = self._compute_tf(msg.poses[i].position, msg.poses[i].orientation, msg.header.frame_id)

                # Send the transformation
                self.br.sendTransform(t)


    def _compute_tf(
        self,
        position: Point,
        orientation: Quaternion,
        frame: str
    ) -> Transform:
        """
        Computes the transform between the marker and the reference frame.

        Args
        ----
            position: The position of the marker in the given frame.
            orientation: The orientation of the marker in the given frame.
            frame: The frame specified in the message.

        Returns
        -------
            transform: The transformation between marker frame and the reference frame.

        """
        now = rclpy.time.Time()
        transform = Transform()

        Tf_frameINreference = self.tf_buffer.lookup_transform(self.reference_frame, frame, now)
        T_frameINreference = homogeneous_matrix(Tf_frameINreference.transform.translation,
                                                    Tf_frameINreference.transform.rotation)
        T_objectINframe = homogeneous_matrix(position, orientation)
        T_objectINreference = T_frameINreference@T_objectINframe
        
        transform.translation.x = transformations.translation_from_matrix(T_objectINreference)[0]
        transform.translation.y = transformations.translation_from_matrix(T_objectINreference)[1]
        transform.translation.z = transformations.translation_from_matrix(T_objectINreference)[2]
        transform.rotation.x = transformations.quaternion_from_matrix(T_objectINreference)[0]
        transform.rotation.y = transformations.quaternion_from_matrix(T_objectINreference)[1]
        transform.rotation.z = transformations.quaternion_from_matrix(T_objectINreference)[2]
        transform.rotation.w = transformations.quaternion_from_matrix(T_objectINreference)[3]

        return transform


def main():
    rclpy.init()
    node = MarkerPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
