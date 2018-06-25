#ifndef CE30_DRIVERS_CE30_D_UTILS_H_
#define CE30_DRIVERS_CE30_D_UTILS_H_

/**
 * @file utils.h
 * @brief helper functions
 */

#include <vector>
#include "udp_socket.h"
#include "ce30_drivers_export.h"
#include <sstream>
#include "common_utils.h"

namespace ce30_drivers {
namespace model_d {
/**
* @brief retrieve device firmware version
* @param[out] version version string
* @param socket UDP socket object
* @return true if successfully retrieved the version; otherwise, false
*/
bool CE30_DRIVERS_API GetVersion(std::string& version, UDPSocket& socket);

/**
 * @brief retrieve device ID [under development]
 * @param id device ID
 * @param socket UDP socket object
 * @return true if successfully retrieved device ID; otherwise, false
 */
bool CE30_DRIVERS_API GetDeviceID(int& id, UDPSocket& socket);

/**
 * @brief set device ID [under development]
 * @param id device ID
 * @param socket UDP socket object
 * @return true if successfully set device ID; otherwise, false
 */
bool CE30_DRIVERS_API SetDeviceID(const int& id, UDPSocket& socket);

/**
 * @brief start receiving measurement data packets
 * @param socket UDP socket object
 * @return true if started; otherwise, false
 */
bool CE30_DRIVERS_API StartRunning(UDPSocket& socket);

/**
 * @brief stop receiving measurement data packets
 * @param socket UDP socket object
 * @return true if stopped; otherwise, false
 */
bool CE30_DRIVERS_API StopRunning(UDPSocket& socket);

/**
 * @brief send packet
 * @param packet the packet to send
 * @param socket UDP socket object
 * @param thread_safe whether to use thread safe mode
 * @return true if sent packet successfully; otherwise false
 */
bool CE30_DRIVERS_API SendPacket(
    const PacketBase& packet, UDPSocket& socket,
    const bool& thread_safe = false);

/// @cond DO_NO_DOCUMENT_THIS
bool CE30_DRIVERS_API EnableFilter(UDPSocket& socket);
/// @endcond
///
/// @cond DO_NO_DOCUMENT_THIS
bool CE30_DRIVERS_API DisableFilter(UDPSocket& socket);
/// @endcond

/// @cond DO_NOT_DOCUMENT_THIS
template<typename T>
std::vector<std::string> B4ToByteHexStrs(const T& t) {
  std::vector<std::string> result(4, "");
  if (sizeof(t) != 4) {
    result.clear();
    return result;
  }

  T b3 = (t & 0xFF000000) >> (8 * 3);
  T b2 = (t & 0x00FF0000) >> (8 * 2);
  T b1 = (t & 0x0000FF00) >> (8 * 1);
  T b0 = (t & 0x000000FF) >> (8 * 0);

  std::stringstream ss;

  ss << std::hex << b0;
  result[0] = ss.str();
  ss.str("");

  ss << std::hex << b1;
  result[1] = ss.str();
  ss.str("");

  ss << std::hex << b2;
  result[2] = ss.str();
  ss.str("");

  ss << std::hex << b3;
  result[3] = ss.str();

  return result;
}
/// @endcond
} // namespace model_d
} // namespace ce30_drivers

#endif // CE30_DRIVERS_CE30_D_UTILS_H_
