"""Broadcast the tool frame to TF."""

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

from threading import Thread

from abb_robot_msgs.srv import GetMechanicalUnitRobTarget
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from tf2_ros.transform_broadcaster import TransformBroadcaster


class ToolFramePublisher(Node):
   """
   Broadcast transforms that never change.

   This node publishes transforms from `base_frame` to the tool frame.
   The transforms are only published once at startup, and are constant for all
   time.

   """

   def __init__(self):
      super().__init__('tool_broadcaster_node')

      self.declare_parameter('from_frame', 'tool0')
      self.declare_parameter('to_frame', 'calibration_tool')
      self.declare_parameter('base_frame', 'base_link')
      self.declare_parameter('rws_tool', False)
      self.declare_parameter('mechanical_unit', 'ROB_R')
      self.declare_parameter('oX', 0.0)
      self.declare_parameter('oY', 0.0)
      self.declare_parameter('oZ', 0.0)

      self.from_frame = self.get_parameter('from_frame').get_parameter_value().string_value
      self.to_frame = self.get_parameter('to_frame').get_parameter_value().string_value
      self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
      self.rws_tool = self.get_parameter('rws_tool').get_parameter_value().bool_value
      self.mech_unit = self.get_parameter('mechanical_unit').get_parameter_value().string_value
      self.offset_x = self.get_parameter('oX').get_parameter_value().double_value
      self.offset_y = self.get_parameter('oY').get_parameter_value().double_value
      self.offset_z = self.get_parameter('oZ').get_parameter_value().double_value

      # create the client
      self.client = self.create_client(GetMechanicalUnitRobTarget, '~/get_mechunit_robtarget')
      while not self.client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info('RAPID service not available, waiting again...')
      self.request = GetMechanicalUnitRobTarget.Request()

      self._tf_publisher = TransformBroadcaster(self)

   def make_transforms(self):
      """Publish the transformation between the desired frames."""
      response = self.send_request()

      transformStamped = TransformStamped()
      transformStamped.header.stamp = self.get_clock().now().to_msg()
      transformStamped.child_frame_id = self.to_frame
      if self.rws_tool:
         transformStamped.header.frame_id = self.base_frame
         transformStamped.transform.translation.x =\
            response.robtarget.trans.x/1000 + float(self.offset_x)
         transformStamped.transform.translation.y =\
            response.robtarget.trans.y/1000 + float(self.offset_y)
         transformStamped.transform.translation.z =\
            response.robtarget.trans.z/1000 + float(self.offset_z)
         transformStamped.transform.rotation.x = response.robtarget.rot.q2
         transformStamped.transform.rotation.y = response.robtarget.rot.q3
         transformStamped.transform.rotation.z = response.robtarget.rot.q4
         transformStamped.transform.rotation.w = response.robtarget.rot.q1
      else:
         transformStamped.header.frame_id = self.from_frame
         transformStamped.transform.translation.x = float(self.offset_x)
         transformStamped.transform.translation.y = float(self.offset_y)
         transformStamped.transform.translation.z = float(self.offset_z)
         transformStamped.transform.rotation.x = 0.0
         transformStamped.transform.rotation.y = 0.0
         transformStamped.transform.rotation.z = 0.0
         transformStamped.transform.rotation.w = 1.0

      self._tf_publisher.sendTransform(transformStamped)

   def send_request(self) -> Future:
      """Call to the service to get the robtarget object."""
      self.request.mechunit = self.mech_unit
      response = self.client.call(self.request)
      return response


def main():
   rclpy.init()
   tool_publisher = ToolFramePublisher()

   spin_thread = Thread(target=rclpy.spin, args=(tool_publisher,))
   spin_thread.start()

   while rclpy.ok():
      tool_publisher.make_transforms()

   rclpy.shutdown()


if __name__ == '__main__':
   main()