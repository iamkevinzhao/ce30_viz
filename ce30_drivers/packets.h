#ifndef CE30_DRIVERS_PACKETS_H_
#define CE30_DRIVERS_PACKETS_H_

#include "ce30_drivers_export.h"
#include <chrono>
#include <vector>

namespace ce30_drivers {
struct CE30_DRIVERS_API PacketBase {
  virtual ~PacketBase();
  /**
   * @brief time stamp when the packet is received
   */
  std::chrono::high_resolution_clock::time_point stamp;
  /**
   * @brief raw packet message in ASCIIs
   */
  std::vector<unsigned char> data;
};
} // namespace ce30_drivers

#endif // CE30_DRIVERS_PACKETS_H_
