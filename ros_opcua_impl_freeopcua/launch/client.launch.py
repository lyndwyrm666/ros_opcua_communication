from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        namespace="opcua",
        node_name="opcua_client",
        package="ros_opcua_impl_freeopcua",
        node_executable="opcua_client",
        output="screen")
])
