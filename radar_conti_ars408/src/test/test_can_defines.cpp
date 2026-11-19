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

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
  #include <catch2/catch_approx.hpp>
using Catch::Approx;
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include <cstdint>

// The header expects these typedefs from the component header
typedef uint8_t ubyte;
typedef uint16_t uword;

#include "../include/ars_408_can_defines.h"

TEST_CASE("Obj_DistLat uses the DBC offset of -204.6 m", "[ars408]")
{
  auto decode = [](int raw) {
      ubyte buf[8] = {0};
      SET_Obj_1_General_Obj_DistLat(buf, raw);
      return CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(buf), 1.0);
    };

  // Raw 1023 is the zero point (START_ value in the DBC)
  CHECK(decode(1023) == Approx(0.0).margin(1e-9));
  CHECK(decode(0) == Approx(-204.6).margin(1e-9));
  CHECK(decode(2047) == Approx(204.8).margin(1e-9));
}
