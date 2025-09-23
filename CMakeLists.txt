cmake_minimum_required(VERSION 3.8)
project(turtlebot_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig)
find_package(posture_analysis_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

pkg_check_modules(PC_ZeroMQ QUIET zmq)
find_path(ZeroMQ_INCLUDE_DIR
          NAMES zmq.hpp
          PATHS ${PC_ZeroMQ_INCLUDE_DIR}
         )
find_library(ZeroMQ_LIBRARY
            NAMES zmq
            PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
            )
include_directories(
  include/turtlebot_bt
)

install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})

add_executable(executable src/exec.cpp)

target_link_libraries(executable BT::behaviortree_cpp_v3)
target_include_directories(executable PUBLIC ${ZeroMQ_INCLUDE_DIRS})
target_link_libraries(executable zmq)
set(dependencies
rclcpp
rclcpp_action
rclcpp_lifecycle
std_msgs
geometry_msgs
nav_msgs
nav2_msgs
behaviortree_cpp_v3
std_srvs
posture_analysis_msgs
)

ament_target_dependencies(executable ${dependencies})

install(TARGETS 
  executable
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
