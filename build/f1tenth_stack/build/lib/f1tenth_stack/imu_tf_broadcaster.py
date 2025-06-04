#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id    = 'base_link'
        t.child_frame_id     = 'imu_link'
        t.transform.translation.x = 0.075
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0719
        # +90° about Z-axis quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.70710678
        t.transform.rotation.w = 0.70710678
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
