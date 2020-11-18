from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ros_opcua_impl_freeopcua",
        node_namespace="opcua", 
        node_name="opcua_client",
        node_executable="client_node",
        output="screen")
])
