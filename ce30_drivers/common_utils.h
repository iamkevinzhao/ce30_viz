#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include "ce30_drivers_export.h"
#include "udp_socket.h"

namespace ce30_drivers {
bool CE30_DRIVERS_API Connect(UDPSocket& socket);

bool CE30_DRIVERS_API GetPacket(
    PacketBase& packet, UDPSocket& socket, const bool& thread_safe = false);
}

#endif // COMMON_UTILS_H
