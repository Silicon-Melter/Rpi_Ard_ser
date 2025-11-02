#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32MultiArray
import serial, time, math

# ===== Terminal colors =====
WHITE  = "\033[97m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
RED    = "\033[91m"
RESET  = "\033[0m"

def xor_checksum(s: str) -> int:
    x = 0
    for b in s.encode('ascii'):
        x ^= b
    return x & 0xFF

def quat_to_roll_yaw_deg(q):
    # Using aerospace sequence (roll about X, yaw about Z). Frame: ENU from MAVROS.
    # roll
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    # wrap to [-180,180]
    if yaw > 180.0:
        yaw -= 360.0
    if yaw < -180.0:
        yaw += 360.0
    return roll, yaw

class SendArduino(Node):
    def __init__(self):
        super().__init__('send_arduino')

        # ===== Parameters =====
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('echo_mavros', False)  # if True: print only, do not send over serial

        self.port       = self.get_parameter('port').value
        self.baud       = int(self.get_parameter('baud').value)
        self.rate_hz    = float(self.get_parameter('rate_hz').value)
        self.echo_only  = bool(self.get_parameter('echo_mavros').value)

        # ===== Serial =====
        self.ser = None
        if not self.echo_only:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.05)
                self.get_logger().info(f"Serial connected on {self.port} @ {self.baud}")
                time.sleep(2.0)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception as e:
                self.get_logger().error(f"Failed to open serial: {e}")
                self.ser = None

        # ===== State =====
        self.v = 0.0            # m/s (body X)
        self.yaw_deg = 0.0      # deg, ENU, wrapped [-180,180]
        self.roll_deg = 0.0     # deg
        self.lpwm = 1500        # µs (always included)
        self.rpwm = 1500        # µs

        # ===== Subscriptions =====
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_body',
            self.cb_velocity, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.cb_pose, qos_profile_sensor_data
        )
        # Optional external PWM source (kept always-on, defaults neutral)
        self.create_subscription(
            Int32MultiArray, '/pwm_cmd', self.cb_pwm, 10
        )

        # ===== Timers =====
        self.create_timer(1.0 / max(1.0, self.rate_hz), self.tick_send)
        if self.ser:
            self._rx_buf = bytearray()
            self.create_timer(0.005, self.tick_read_serial)

        self.get_logger().info(
            f"Ready — MAVROS printed in white, Arduino in yellow. "
            f"{'ECHO-ONLY (no serial tx)' if self.echo_only else 'TX active'}."
        )

    # ===== Callbacks =====
    def cb_velocity(self, msg: TwistStamped):
        self.v = float(msg.twist.linear.x)
        print(f"{WHITE}MAVROS v={self.v:.3f} m/s{RESET}")

    def cb_pose(self, msg: PoseStamped):
        roll, yaw = quat_to_roll_yaw_deg(msg.pose.orientation)
        self.roll_deg = roll
        self.yaw_deg  = yaw
        print(f"{WHITE}MAVROS roll={self.roll_deg:.2f}° yaw={self.yaw_deg:.2f}°{RESET}")

    def cb_pwm(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.lpwm = int(msg.data[0])
            self.rpwm = int(msg.data[1])

    # ===== TX: Pi → Arduino =====
    def tick_send(self):
        if self.echo_only:
            return
        if not self.ser or not self.ser.is_open:
            return

        now_ms = int(time.time() * 1000)
        # Packet: S,v=...,yaw=...,roll=...,lpwm=...,rpwm=...,ts=...*CS\n
        body = (f"S,v={self.v:.3f},yaw={self.yaw_deg:.2f},roll={self.roll_deg:.2f},"
                f"lpwm={self.lpwm},rpwm={self.rpwm},ts={now_ms}")
        cs = xor_checksum(body)
        pkt = f"{body}*{cs:02X}\n"
        try:
            self.ser.write(pkt.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ===== RX: Arduino → Pi (monitoring) =====
    def tick_read_serial(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            data = self.ser.read(512)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        if not data:
            return

        if not hasattr(self, "_rx_buf"):
            self._rx_buf = bytearray()
        self._rx_buf.extend(data)

        while True:
            nl = self._rx_buf.find(b'\n')
            if nl < 0:
                break
            line = self._rx_buf[:nl].decode('utf-8', errors='replace').strip()
            del self._rx_buf[:nl+1]
            if line:
                print(f"{YELLOW}ARDUINO → {line}{RESET}")

def main(args=None):
    rclpy.init(args=args)
    node = SendArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{GREEN}\nShutting down cleanly...{RESET}")
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
