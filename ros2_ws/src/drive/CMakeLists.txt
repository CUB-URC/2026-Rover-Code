cmake_minimum_required(VERSION 3.8)
project(drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add executables
add_executable(drive_controller_node src/drive_controller.cpp)
ament_target_dependencies(drive_controller_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  ament_index_cpp
  yaml-cpp
)
target_link_libraries(drive_controller_node yaml-cpp)

# Add simulation executable
add_executable(drive_controller_sim src/drive_controller_sim.cpp)
ament_target_dependencies(drive_controller_sim
  rclcpp 
  std_msgs 
  geometry_msgs 
  ament_index_cpp
  yaml-cpp
)
target_link_libraries(drive_controller_sim yaml-cpp)

# Add test executable
add_executable(drive_test_node src/drive_test.cpp)
ament_target_dependencies(drive_test_node 
  rclcpp 
  geometry_msgs
)

# Add Xbox controller executable
add_executable(xbox_controller_node src/xbox_controller_node.cpp)
ament_target_dependencies(xbox_controller_node
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS drive_controller_node drive_controller_sim drive_test_node xbox_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
