#ifndef RADAR_CONTI_ARS408_STRUCTS_HPP
#define RADAR_CONTI_ARS408_STRUCTS_HPP

#include <cstdint>

namespace radar_conti_ars408_structs
{
  struct MotionInputSignal
  {
    double speed;
    double yaw_rate;
    uint8_t direction;
  };
}

#endif // RADAR_CONTI_ARS408_STRUCTS_HPP