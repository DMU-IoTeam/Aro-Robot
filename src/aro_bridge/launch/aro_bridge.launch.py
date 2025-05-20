from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pi_ip_arg = LaunchConfiguration("pi_ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pi_ip",
            default_value="100.64.163.104",
            description="Tailscale IP of Raspberry Pi rosbridge server",
        ),

        Node(
            package="aro_bridge",
            executable="bridge_node",
            name="bridge_node",
            parameters=[{"pi_ip": pi_ip_arg}],
            output="screen",
        ),
    ])