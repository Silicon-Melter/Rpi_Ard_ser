#!/usr/bin/env python3
import rclpy, time, math, serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32MultiArray

WHITE  = "\033[97m"; YELLOW = "\033[93m"; GREEN  = "\033[92m"; RESET  = "\033[0m"

def xor_checksum(s: str) -> int:
    x = 0
    for b in s.encode('ascii'): x ^= b
    return x & 0xFF

class VelocityPoseSender(Node):
    def __init__(self):
        super().__init__('velocity_pose_sender')

        # Params
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.05)
            self.get_logger().info(f"Serial connected on {self.port} @ {self.baud}")
            time.sleep(2.0)
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self.ser = None

        # State
        self.v = 0.0; self.w = 0.0; self.roll_deg = 0.0
        self.lpwm = 1500; self.rpwm = 1500  # always included

        # Subs
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_body',
                                 self.cb_velocity, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.cb_pose, qos_profile_sensor_data)
        self.create_subscription(Int32MultiArray, '/pwm_cmd', self.cb_pwm, 10)

        # Timers
        self.create_timer(1.0 / max(1.0, self.rate_hz), self.tick_send)
        self.create_timer(0.005, self.tick_read_serial)

        self._rx_buf = bytearray()
        self.get_logger().info("Ready — MAVROS (white), Arduino (yellow). PWM always included.")

    def cb_velocity(self, msg: TwistStamped):
        self.v = float(msg.twist.linear.x)
        self.w = float(msg.twist.angular.z)
        print(f"{WHITE}MAVROS v={self.v:.3f} m/s, w={self.w:.3f} rad/s{RESET}")

    def cb_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        sinr_cosp = 2.0*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
        self.roll_deg = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        print(f"{WHITE}MAVROS roll={self.roll_deg:.2f}°{RESET}")

    def cb_pwm(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.lpwm = int(msg.data[0]); self.rpwm = int(msg.data[1])

    def tick_send(self):
        if not self.ser or not self.ser.is_open: return
        now_ms = int(time.time() * 1000)
        body = (f"S,v={self.v:.3f},w={self.w:.3f},roll={self.roll_deg:.2f},"
                f"lpwm={self.lpwm},rpwm={self.rpwm},ts={now_ms}")
        cs = xor_checksum(body)
        pkt = f"{body}*{cs:02X}\n"
        try:
            self.ser.write(pkt.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def tick_read_serial(self):
        if not self.ser or not self.ser.is_open: return
        try:
            data = self.ser.read(512)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return
        if data: self._rx_buf.extend(data)
        while True:
            nl = self._rx_buf.find(b'\n')
            if nl < 0: break
            line = self._rx_buf[:nl].decode('utf-8', errors='replace').strip()
            del self._rx_buf[:nl+1]
            if line: print(f"{YELLOW}ARDUINO → {line}{RESET}")

def main(args=None):
    rclpy.init(args=args); node = VelocityPoseSender()
    try: rclpy.spin(node)
    except KeyboardInterrupt: print(f"{GREEN}\nShutting down...{RESET}")
    finally:
        if node.ser and node.ser.is_open: node.ser.close()
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

