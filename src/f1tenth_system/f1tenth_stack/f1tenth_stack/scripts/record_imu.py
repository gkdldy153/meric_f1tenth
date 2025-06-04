#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRecorder(Node):
    def __init__(self):
        super().__init__('imu_recorder')
        self.sub = self.create_subscription(
            Imu, '/sensors/imu/raw', self.cb, 10)
        # open CSV and write header
        self.csvfile = open('imu_data.csv', 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'sec', 'nanosec',
            'ax','ay','az',
            'wx','wy','wz',
            'qw','qx','qy','qz'
        ])

    def cb(self, msg: Imu):
        h = msg.header.stamp
        row = [
            h.sec, h.nanosec,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]
        self.writer.writerow(row)

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = ImuRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
