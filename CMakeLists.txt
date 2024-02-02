cmake_minimum_required(VERSION 3.8)
project(dummy_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wunused-variable)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
generate_parameter_library(dummy_odometry_node_parameters
  src/dummy_odometry_node_parameters.yaml
)

add_library(dummy_odometry SHARED)
target_sources(dummy_odometry
  PRIVATE
    src/simple_dynamics.cpp
    src/state_space/mdk.cpp
)
target_link_libraries(dummy_odometry
  PUBLIC
    Eigen3::Eigen
)
target_compile_features(dummy_odometry
  PRIVATE
    cxx_std_20
)
target_include_directories(dummy_odometry
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_auto_add_library(dummy_odometry_components SHARED
  src/dummy_odometry_node.cpp
)
target_link_libraries(dummy_odometry_components
  Eigen3::Eigen
  dummy_odometry_node_parameters
  dummy_odometry
)
target_include_directories(dummy_odometry_components
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(dummy_odometry_components
  PRIVATE
    cxx_std_17
)
rclcpp_components_register_node(dummy_odometry_components
  PLUGIN "dummy_odometry::DummyOdometryNode"
  EXECUTABLE dummy_odometry_node
)

install(
  TARGETS
    dummy_odometry
    dummy_odometry_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
