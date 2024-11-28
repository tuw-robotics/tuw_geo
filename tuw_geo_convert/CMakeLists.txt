cmake_minimum_required(VERSION 3.8)
project(tuw_geo_convert)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tuw REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib QUIET)
if(GeographicLib_FOUND)
  message(STATUS "GeographicLib found: building geo_map!")
else()
  message(ERROR "GeographicLib NOT found: not building geo_map!")
endif()


add_executable(geo_convert_node src/geo_convert_node.cpp src/standalone_geo_convert.cpp)
target_include_directories(geo_convert_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(geo_convert_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  geo_convert_node
  "rclcpp"
  "tf2_ros"
  "nav_msgs"
  "sensor_msgs"
  "tuw"
  GeographicLib
)

add_executable(time_fix_node src/time_fix_node.cpp src/standalone_time_fix.cpp)
target_include_directories(time_fix_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(time_fix_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  time_fix_node
  "rclcpp"
  "tf2_ros"
  "nav_msgs"
  "sensor_msgs"
  "tuw"
)

install(TARGETS geo_convert_node time_fix_node
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
