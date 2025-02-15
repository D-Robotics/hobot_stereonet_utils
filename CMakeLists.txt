# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(hobot_stereonet_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(save_stereo_img src/save_stereo_img.cpp)
target_include_directories(save_stereo_img PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(save_stereo_img PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(save_stereo_img
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(save_depth_img src/save_depth_img.cpp)
target_include_directories(save_depth_img PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(save_depth_img PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(save_depth_img
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(pub_rectify_img src/pub_rectify_img.cpp)
target_include_directories(pub_rectify_img PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pub_rectify_img PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pub_rectify_img
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS save_stereo_img DESTINATION lib/${PROJECT_NAME})
install(TARGETS save_depth_img DESTINATION lib/${PROJECT_NAME})
install(TARGETS pub_rectify_img DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
