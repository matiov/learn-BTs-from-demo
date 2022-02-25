"""
Create a static tf from map to yumi
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
   """
   Broadcast transforms that never change.
   """

   def __init__(self):
        super().__init__('static_yumi_tf_broadcaster')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('oX', 1.80)
        self.declare_parameter('oY', 2.30)
        self.declare_parameter('oZ', 0.82)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.offset_x = self.get_parameter('oX').get_parameter_value().double_value
        self.offset_y = self.get_parameter('oY').get_parameter_value().double_value
        self.offset_z = self.get_parameter('oZ').get_parameter_value().double_value

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms()

   def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'myumi_003_map'
        static_transformStamped.child_frame_id = 'wasp_yumi_base_link'
        static_transformStamped.transform.translation.x = float(self.offset_x)
        static_transformStamped.transform.translation.y = float(self.offset_y)
        static_transformStamped.transform.translation.z = float(self.offset_z)
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = -0.7071068
        static_transformStamped.transform.rotation.w = 0.7071068

        self._tf_publisher.sendTransform(static_transformStamped)


def main():
    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()