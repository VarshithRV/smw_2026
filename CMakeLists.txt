cmake_minimum_required(VERSION 3.8)
project(smw_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sut06_dual_arm_workcell_bringup REQUIRED)
find_package(motion_planning_abstractions_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(motion_planning_abstractions REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  std_srvs
  motion_planning_abstractions
  motion_planning_abstractions_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  trajectory_msgs
  ur_msgs
  rosidl_runtime_cpp
  trajectory_msgs
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

add_executable(perception_proxy_server src/perception_proxy_server.cpp)
ament_target_dependencies(perception_proxy_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(perception_proxy_server PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
perception_proxy_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(co1_deconing src/CO1_deconing.cpp)
ament_target_dependencies(co1_deconing ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(co1_deconing PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
co1_deconing
DESTINATION lib/${PROJECT_NAME}
)

add_executable(co4_deconing src/C04_deconing.cpp)
ament_target_dependencies(co4_deconing ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(co4_deconing PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
co4_deconing
DESTINATION lib/${PROJECT_NAME}
)

add_executable(so1_deconing src/S01_deconing.cpp)
ament_target_dependencies(so1_deconing ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(so1_deconing PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
so1_deconing
DESTINATION lib/${PROJECT_NAME}
)

add_executable(so3_deconing src/S03_deconing.cpp)
ament_target_dependencies(so3_deconing ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(so3_deconing PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
so3_deconing
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
