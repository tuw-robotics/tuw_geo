cmake_minimum_required(VERSION 3.8)
project(tuw_geo_objects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tuw_geometry REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib QUIET)
if(GeographicLib_FOUND)
  message(STATUS "GeographicLib found: building geo_map!")
else()
  message(ERROR "GeographicLib NOT found: not building geo_map!")
endif()

add_library(geo_objects src/geo_objects_node.cpp)
target_compile_features(geo_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(geo_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  geo_objects
  "rclcpp"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "tf2_ros"
  "nav_msgs"
  "tuw_geometry"
  "OpenCV"
)

add_executable(geo_objects_node src/standalone_geo_objects.cpp)
target_include_directories(geo_objects_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(geo_objects_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(geo_objects_node geo_objects )
ament_target_dependencies(
  geo_objects_node
  "rclcpp"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "tf2_ros"
  "nav_msgs"
  "tuw_geometry"
  "OpenCV"
)

install(TARGETS 
  geo_objects_node
  geo_objects
  DESTINATION lib/${PROJECT_NAME})

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
