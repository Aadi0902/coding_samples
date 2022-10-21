cmake_minimum_required(VERSION 3.8)
project(raceline_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bvs_utils REQUIRED)
find_package(bvs_msgs REQUIRED)
find_package(path_utils REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(system_executive REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_package()

add_executable(raceline_planner_node src/raceline_planner_node.cpp)
# target_link_libraries(raceline_planner_node lqr_control bvs_control_utils Eigen3::Eigen)
ament_target_dependencies(raceline_planner_node rclcpp bvs_utils std_msgs nav_msgs geometry_msgs bvs_msgs path_utils system_executive deep_orange_msgs)

add_executable(dummy_car_sim src/dummy_car_sim.cpp)
# target_link_libraries(raceline_planner_node lqr_control bvs_control_utils Eigen3::Eigen)
ament_target_dependencies(dummy_car_sim rclcpp bvs_utils std_msgs nav_msgs geometry_msgs bvs_msgs path_utils)

add_executable(merged_path_node src/merged_path_node.cpp)
ament_target_dependencies(merged_path_node rclcpp bvs_utils std_msgs nav_msgs geometry_msgs bvs_msgs path_utils)

add_executable(ghost_node src/ghost_node.cpp)
ament_target_dependencies(ghost_node rclcpp bvs_utils std_msgs nav_msgs geometry_msgs bvs_msgs path_utils)

add_executable(keyboard_set_state_node src/keyboard_set_state_node.cpp)
ament_target_dependencies(keyboard_set_state_node rclcpp std_msgs)

install(
  TARGETS raceline_planner_node dummy_car_sim keyboard_set_state_node merged_path_node ghost_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package(INSTALL_TO_SHARE
  launch
)
