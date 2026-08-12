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
cmake_minimum_required(VERSION 3.5)
project(radar_conti_ars408_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()#(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(radar_conti_ars408_msgs
  "msg/ClusterGeneral.msg"
  "msg/ClusterQuality.msg"
  "msg/ClusterStatus.msg"
  "msg/ObjectGeneral.msg"
  "msg/ObjectExtended.msg"
  "msg/ObjectQuality.msg"
  "msg/ObjectStatus.msg"
  "msg/RadarConfiguration.msg"
  "msg/RadarState.msg"
  "msg/VersionID.msg"
  "msg/Cluster.msg"
  "msg/Object.msg"
  "msg/ClusterList.msg"
  "msg/ObjectList.msg"
  "msg/FilterStateCfgEntryFloat.msg"
  "msg/FilterStateCfgEntryInt.msg"
  "msg/FilterStateCfg.msg"
  "msg/FilterCfg.msg"
  "msg/FilterStateHeader.msg"
  "srv/SetFilter.srv"
  "srv/TriggerSetCfg.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
endif()

ament_package()
