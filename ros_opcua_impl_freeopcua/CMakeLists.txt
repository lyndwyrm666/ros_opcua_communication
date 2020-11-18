cmake_minimum_required(VERSION 3.5)
project(ros_opcua_impl_freeopcua)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wpedantic -Wextra)
endif()

set(BUILD_CLIENT ON)
set(BUILD_SERVER ON)
set(BUILD_TESTING OFF)

add_subdirectory(freeopcua)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_opcua_msgs REQUIRED)
find_package(ros_opcua_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

include_directories(
  include
  freeopcua/include
  ${Boost_INCLUDE_DIRS}
)

ament_export_dependencies(rclcpp ros_opcua_msgs ros_opcua_srvs)
ament_export_include_directories(include freeopcua/include)
ament_export_libraries(opcua opcuaprotocol opcuacore opcuaclient opcuaserver)

add_executable(client_node src/opcua_client.cpp)
ament_target_dependencies(client_node ros_opcua_msgs ros_opcua_srvs rclcpp)
target_link_libraries(client_node ${Boost_LIBRARIES} opcuaprotocol opcuacore opcuaclient opcuaserver)

#############
## Install ##
#############


install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}}
)
install(DIRECTORY include/ DESTINATION include}
)

install(TARGETS client_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}}
  LIBRARY DESTINATION lib/${PROJECT_NAME}}
  RUNTIME DESTINATION lib/${PROJECT_NAME}}
)
ament_package()
