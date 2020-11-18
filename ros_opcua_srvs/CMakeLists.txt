cmake_minimum_required(VERSION 3.5)
project(ros_opcua_srvs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_opcua_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CallMethod.srv"
  "srv/Connect.srv"
  "srv/Disconnect.srv"
  "srv/ListNode.srv"
  "srv/ReadOpc.srv"
  "srv/Subscribe.srv"
  "srv/Unsubscribe.srv"
  "srv/WriteOpc.srv"
  DEPENDENCIES ros_opcua_msgs std_msgs std_srvs
)
ament_export_dependencies(ros_opcua_msgs std_msgs std_srvs rosidl_default_runtime)
ament_package()
