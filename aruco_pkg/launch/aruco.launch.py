from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare command line argument for launching the aruco_sensor node
    declare_aruco_sensor_cmd = DeclareLaunchArgument(
        'aruco_sensor',
        default_value='True',
        description='Whether to launch the aruco_sensor node'
    )

    # Define the aruco_sensor node with conditional launching
    aruco_sensor_node = Node(
        package='aruco_pkg',
        namespace='namespace1',
        executable='aruco_sensor',
        name='aruco_sensor',
        condition=IfCondition(LaunchConfiguration('aruco_sensor'))
    )

    return LaunchDescription([
        declare_aruco_sensor_cmd,
        aruco_sensor_node,
    ])
