cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# optional, skipped if gazebo harmonic isn't installed
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET COMPONENTS register)
if(gz-sim8_FOUND AND gz-plugin2_FOUND)
  add_library(RockerDifferential SHARED src/RockerDifferential.cc)
  target_compile_features(RockerDifferential PRIVATE cxx_std_17)
  target_link_libraries(RockerDifferential gz-sim8::gz-sim8 gz-plugin2::register)
  install(TARGETS RockerDifferential LIBRARY DESTINATION lib)
else()
  message(WARNING "Gazebo Harmonic (gz-sim8 / gz-plugin2) not found - skipping the "
                  "RockerDifferential plugin. Install it with scripts/setup_sim.sh or use the sim container.")
endif()

install(DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
