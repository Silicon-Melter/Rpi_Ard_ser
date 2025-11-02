from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # optional: hard-pin the USB port to avoid serial parsing weirdness
    usb_port_id = LaunchConfiguration("usb_port_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "usb_port_id",
            default_value="2-1",  # update if your port differs (see rs-enumerate-devices logs)
            description="USB port path for the RealSense device (e.g. 2-1 or 2-2)"
        ),

        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            output="screen",
            parameters=[{
                # device selection
                "usb_port_id": usb_port_id,

                # stability-friendly defaults for Pi
                "initial_reset": True,
                "enable_sync": False,
                "publish_tf": False,

                # streams
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "align_depth": True,

                # profiles (keep bandwidth modest)
                "rgb_camera.color_profile": "640x480x15",
                "depth_module.depth_profile": "640x360x15",
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                # default port 8765; change if you want
                "port": 8765,
                "use_compression": True
            }],
            respawn=True,
            respawn_delay=2.0
        ),
    ])
