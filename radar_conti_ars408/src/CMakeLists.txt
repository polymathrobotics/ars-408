# Copyright 2026 Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.22)

project(radar_conti_ars408)

# Default to C++14
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ars_lib ${PROJECT_NAME}_component)
set(ars_bin ${PROJECT_NAME}_composition)

add_library(${ars_lib} SHARED src/radar_conti_ars408_component.cpp)
target_compile_definitions(${ars_lib} PRIVATE "RADAR_CONTI_ARS408_BUILDING_DLL")
target_include_directories(${ars_lib} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include"
)
target_link_libraries(${ars_lib} dl -lboost_system)

set_source_files_properties(include/ars_408_can_defines.h PROPERTIES COMPILE_FLAGS "-Wno-type-limits")

ament_target_dependencies(${ars_lib}
  ament_index_cpp
  bond
  bondcpp
  can_msgs
  class_loader
  geometry_msgs
  nav2_dynamic_msgs
  nav_msgs
  radar_conti_ars408_msgs
  radar_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  socketcan_adapter
  std_msgs
  tf2_ros
  visualization_msgs
)

rclcpp_components_register_nodes(${ars_lib} "FHAC::radar_conti_ars408")
set(node_plugins "FHAC::radar_conti_ars408;$<TARGET_FILE:radar_conti_ars408_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


add_executable(${ars_bin} src/radar_conti_ars408_composition.cpp)
target_link_libraries(${ars_bin} ${ars_lib})

ament_target_dependencies(${ars_bin}
  "rclcpp"
)

install(
  TARGETS ${ars_bin}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS ${ars_lib}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  find_package(Catch2 REQUIRED)

  add_executable(test_radar_transforms test/test_radar_transforms.cpp)
  target_link_libraries(test_radar_transforms Catch2::Catch2WithMain ${ars_lib})
  add_test(NAME test_radar_transforms COMMAND test_radar_transforms)

  # Until CI has VCAN support, don't run this test
  if(NOT DEFINED ENV{CI})
    add_executable(test_component test/test_radar_conti_ars408_component.cpp)
    target_link_libraries(test_component Catch2::Catch2WithMain ${ars_lib})
    add_test(NAME test_component COMMAND test_component)
  endif()
endif()

ament_package()
