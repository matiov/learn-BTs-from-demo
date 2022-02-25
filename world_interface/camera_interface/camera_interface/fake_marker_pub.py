"""Publish the pose of some markers for testing purposes."""

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
from geometry_msgs.msg import Pose

import numpy as np

import rclpy
import rclpy.node


class MarkerPub(rclpy.node.Node):
    """This node publishes a marker pose to the tf tree."""

    def __init__(self):
        super().__init__('fake_marker_publisher')

        self.pose_pub = []
        self.marker_IDs = [53, 20, 41, 35, 26, 47]  # A,B,C,D,E,F

        # Parameters bounds in m
        n = len(self.marker_IDs)  # number of points
        ax = 0.5  # upper bound x
        bx = -0.5  # lower bound x
        ay = 1.0  # upper bound y
        by = 0.1  # lower bound y
        # Random coordinates [b,a) uniform distributed
        self.coordy = (by - ay) * np.random.random_sample((n,)) + ay  # generate random y
        self.coordx = (bx - ax) * np.random.random_sample((n,)) + ax  # generate random x
        self.coordz = 0.05/2  # the cube is 5cm so the centroid is at half of its heigth

        # Set up publishers
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # create message
        markers = ArucoMarkers()
        markers.header.frame_id = 'base_link'
        markers.header.stamp = self.get_clock().now().to_msg()
        # generate the poses to be published
        for i, marker in enumerate(self.marker_IDs):
            # all fake markers are in base frame
            pose = Pose()
            # position
            pose.position.x = float(self.coordx[i])
            pose.position.y = float(self.coordy[i])
            pose.position.z = float(self.coordz)
            # orientation
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            markers.poses.append(pose)
            markers.marker_ids.append(marker)

        self.markers_pub.publish(markers)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = MarkerPub()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
