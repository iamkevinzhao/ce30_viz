#include "packets.h"

namespace ce30_drivers {
// PacketBase
PacketBase::~PacketBase() {}

// ParsedPacket
ParsedPacket::~ParsedPacket() {}

// Packet
Packet::~Packet() {}

std::unique_ptr<ParsedPacket> Packet::ParseBase() {
  return nullptr;
}

// Column
Column::~Column() {}

// Channel
Channel::~Channel() {}
Point Channel::point() const {
  return Point();
}
}
