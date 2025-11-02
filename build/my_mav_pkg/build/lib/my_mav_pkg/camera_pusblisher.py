#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# --- THIS IS THE NUMBER YOU FOUND IN STEP 2 ---
CAMERA_INDEX = 0 
# ---------------------------------------------

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # --- Define QoS Profile ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 
        )

        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
        timer_period = 0.05  # seconds (aims for ~20 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # --- Try opening the camera ---
        self.cap = cv2.VideoCapture(CAMERA_INDEX)

        # Wait for camera to initialize
        time.sleep(1) 

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device /dev/video{CAMERA_INDEX}')
            raise RuntimeError(f"Failed to open camera at index {CAMERA_INDEX}")
        else:
             self.get_logger().info(f'Video device /dev/video{CAMERA_INDEX} opened successfully.')
             # Optional: Set resolution
             # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
             # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.br = CvBridge()
        self.get_logger().info('Webcam Publisher Node started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image (BGR) to ROS Image message
            ros_image = self.br.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().warn(f'Failed to capture frame from /dev/video{CAMERA_INDEX}')

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info('Releasing video capture device.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = None
    try:
        webcam_publisher = WebcamPublisher()
        rclpy.spin(webcam_publisher)
    except RuntimeError as e:
        print(f"Node initialization failed: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if webcam_publisher:
            webcam_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()