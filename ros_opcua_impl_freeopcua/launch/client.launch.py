from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ros_opcua_impl_freeopcua",
        namespace="opcua", 
        name="opcua_client",
        executable="client_node",
        output="screen")
])
