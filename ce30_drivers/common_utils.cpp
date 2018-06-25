#include "common_utils.h"

namespace ce30_drivers {
bool Connect(UDPSocket& socket) {
  return socket.Connect() == Diagnose::connect_successful;
}

bool GetPacket(
    PacketBase& packet, UDPSocket& socket, const bool& thread_safe) {
  Diagnose diagnose;
  if (thread_safe) {
    diagnose = socket.GetPacketThreadSafe(packet);
  } else {
    diagnose = socket.GetPacket(packet);
  }
  return diagnose == Diagnose::receive_successful;
}
}
