from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="TEL",
            executable="wheel_sub",
            name="wheel_sub"
            ),
#        Node(
#            package="TEL",
#            executable="camera_servo_sub",
#            name="camera_sub"
#            ),
        Node(
            package="TEL",
            executable="load1_servo_sub",
            name="load1_sub"
            ),
        Node(
            package="TEL",
            executable="shooter_CIM_sub",
            name="shooter_CIM_sub"
            ),
       Node(
            package="TEL",
            executable="shooter_servo_sub",
            name="shooter_servo_sub"
            ),
        Node(
            package="TEL",
            executable="up_linear_sub",
            name="up_linear_sub"
            ),
        Node(
            package="TEL",
            executable="bottom_linear_sub",
            name="bottom_linear_sub"
            ),
        Node(
            package="TEL",
            executable="server",
            name="server"
            ),
          
        ])

