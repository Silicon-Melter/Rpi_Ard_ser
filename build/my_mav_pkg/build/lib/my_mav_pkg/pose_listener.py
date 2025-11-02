#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import serial
import math

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')

        # === Serial port setup ===
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial connected on /dev/ttyUSB0')
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial: {e}')

        # === ROS2 Subscriber ===
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.subscription
        self.get_logger().info('Listening to /mavros/local_position/pose ...')

    def pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation

        # === Convert quaternion → yaw (heading in degrees) ===
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y**2 + ori.z**2)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        # === Log ===
        self.get_logger().info(
            f'Pos: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f} | Heading: {yaw_deg:.2f}°'
        )

        # === Send heading to serial ===
        if self.serial_port and self.serial_port.is_open:
            msg_str = f"{yaw_deg:.2f}\n"
            self.serial_port.write(msg_str.encode('utf-8'))

    def destroy_node(self):
        # Close serial when shutting down
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
