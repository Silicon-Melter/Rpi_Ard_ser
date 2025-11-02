#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Import both Image and CameraInfo
from sensor_msgs.msg import Image, CameraInfo 
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# --- THIS IS THE NUMBER YOU FOUND IN STEP 2 ---
CAMERA_INDEX = 0 
# ---------------------------------------------
# --- Define your camera's resolution ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
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

        # --- Create BOTH publishers ---
        self.image_publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
        self.info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', qos_profile) # New
        
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
             # --- Set resolution ---
             self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
             self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.br = CvBridge()

        # --- Create a dummy CameraInfo message ---
        self.cam_info_msg = CameraInfo()
        self.cam_info_msg.width = FRAME_WIDTH
        self.cam_info_msg.height = FRAME_HEIGHT
        self.cam_info_msg.header.frame_id = 'camera_frame' # Give it a frame_id
        # Dummy calibration matrix (K) - Assumes lens center is image center
        self.cam_info_msg.k = [float(FRAME_WIDTH), 0.0, float(FRAME_WIDTH / 2),
                               0.0, float(FRAME_HEIGHT), float(FRAME_HEIGHT / 2),
                               0.0, 0.0, 1.0]
        # No distortion
        self.cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Dummy projection matrix (P)
        self.cam_info_msg.p = [float(FRAME_WIDTH), 0.0, float(FRAME_WIDTH / 2), 0.0,
                               0.0, float(FRAME_HEIGHT), float(FRAME_HEIGHT / 2), 0.0,
                               0.0, 0.0, 1.0, 0.0]
        # No rectification
        self.cam_info_msg.r = [1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0]

        self.get_logger().info('Webcam Publisher Node started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Get the current time once
            now_stamp = self.get_clock().now().to_msg()

            # --- Publish Image ---
            ros_image = self.br.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = now_stamp # Apply timestamp
            ros_image.header.frame_id = 'camera_frame' # Apply frame_id
            self.image_publisher_.publish(ros_image)
            
            # --- Publish CameraInfo ---
            self.cam_info_msg.header.stamp = now_stamp # Apply THE SAME timestamp
            self.info_publisher_.publish(self.cam_info_msg)

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