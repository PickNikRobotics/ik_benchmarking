cmake_minimum_required(VERSION 3.8)
project(ik_benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# For adding the IKBenchmark action
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/IKBenchmark.action"
  DEPENDENCIES std_msgs
)

add_executable(ik_benchmarking_server src/ik_benchmarking.cpp src/ik_benchmarking_server.cpp)
add_executable(ik_benchmarking_client src/ik_benchmarking_client.cpp)

target_include_directories(ik_benchmarking_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(ik_benchmarking_client PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_link_libraries(ik_benchmarking_server rclcpp::rclcpp fmt::fmt)
target_link_libraries(ik_benchmarking_client rclcpp::rclcpp)

#link against interfaces (actions) generated in the same project
rosidl_target_interfaces(ik_benchmarking_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(ik_benchmarking_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_compile_features(ik_benchmarking_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ik_benchmarking_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  ik_benchmarking_server
  "moveit_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
)
ament_target_dependencies(
  ik_benchmarking_client
  "moveit_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
)

install(TARGETS
  ik_benchmarking_server
  ik_benchmarking_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ik_benchmarking_data_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
