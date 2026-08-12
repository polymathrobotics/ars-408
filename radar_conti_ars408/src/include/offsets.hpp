// Copyright 2026 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// offsets.hpp
#ifndef OFFSETS_HPP
#define OFFSETS_HPP

// Filter Config

namespace FilterConfig
{

// Resolutions
constexpr double DISTANCE_RESOLUTION = 0.1;
constexpr double AZIMUTH_RESOLUTION = 0.025;
constexpr double VRELONCOME_RESOLUTION = 0.0315;
constexpr double VRELDEPART_RESOLUTION = 0.0315;
constexpr double RCS_RESOLUTION = 0.025;
constexpr double LIFETIME_RESOLUTION = 0.1;
constexpr double SIZE_RESOLUTION = 0.025;
constexpr double Y_RESOLUTION = 0.2;
constexpr double X_RESOLUTION = 0.2;
constexpr double VYRIGHTLEFT_RESOLUTION = 0.0315;
constexpr double VXONCOME_RESOLUTION = 0.0315;
constexpr double VYLEFTRIGHT_RESOLUTION = 0.0315;
constexpr double VXDEPART_RESOLUTION = 0.0315;

// Offsets
constexpr double AZIMUTH_OFFSET = 50.0;
constexpr double RCS_OFFSET = 50.0;
constexpr double Y_OFFSET = 409.5;
constexpr double X_OFFSET = 500;
}  // namespace FilterConfig

// Motion Config
namespace YawRateInformation
{
constexpr double YAW_RATE_OFFSET = 327.68;
constexpr double YAW_RATE_RESOLUTION = 0.01;
}  // namespace YawRateInformation

namespace SpeedInformation
{
constexpr double SPEED_RESOLUTION = 0.02;
}

#endif  // OFFSETS_HPP
