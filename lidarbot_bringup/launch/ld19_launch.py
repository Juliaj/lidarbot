# This launch file runs the ldlidar_stl_ros2 node with the configurations specified
# ref: https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_bringup/launch/lasers.launch.py

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="230400")
    frame_id = LaunchConfiguration("frame_id", default="lidar_link")
    inverted = LaunchConfiguration("inverted", default="false")
    enable_angle_crop_func = LaunchConfiguration(
        "enable_angle_crop_func", default="false"
    )
    angle_crop_min = LaunchConfiguration("angle_crop_min", default="135.0")
    angle_crop_max = LaunchConfiguration("angle_crop_max", default="225.0")
    laser_scan_dir = LaunchConfiguration("laser_scan_dir", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Specifying frame_id of lidar",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "laser_scan_dir",
                default_value=laser_scan_dir,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "enable_angle_crop_func",
                default_value=enable_angle_crop_func,
                description="Specifying whether or not to enable angle crop function",
            ),
            DeclareLaunchArgument(
                "angle_crop_min",
                default_value=angle_crop_min,
                description="Specifying the minimum angle to crop",
            ),
            DeclareLaunchArgument(
                "angle_crop_max",
                default_value=angle_crop_max,
                description="Specifying the maximum angle to crop",
            ),
            # LD19 LiDAR node
            Node(
                package="ldlidar_stl_ros2",
                executable="ldlidar_stl_ros2_node",
                name="ld19",
                parameters=[
                    {"product_name": "LDLiDAR_LD19"},
                    {"topic_name": "scan"},
                    {"frame_id": frame_id},
                    {"port_name": serial_port},
                    {"port_baudrate": serial_baudrate},
                    {"laser_scan_dir": laser_scan_dir},
                    {"enable_angle_crop_func": enable_angle_crop_func},
                    {"angle_crop_min": angle_crop_min},
                    {"angle_crop_max": angle_crop_max},
                ],
                output="screen",
            ),
        ]
    )
