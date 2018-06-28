#include "model_x_packets.h"

namespace ce30_drivers {
namespace model_x {
// Packet
Packet::Packet() {
  data.resize(TotalLength(), 0);
}

std::unique_ptr<ParsedPacket> Packet::Parse() {
  return nullptr;
}

// 1192
int Packet::TotalLength() {
  return 1192;
}

// Scan
Scan::Scan() {}
void Scan::AddFromPacket(const ParsedPacket& packet) {}
bool Scan::Ready() {
  return false;
}

void Scan::Reset() {}

Channel Scan::at(const int& x, const int& y) const {
  Channel channel;
  channel.distance = 0.5f;
  return channel;
}
int Scan::Width() {
  return 320;
}
int Scan::Height() {
  return 20;
}
float Scan::DistanceMin() {
  return Channel::DistanceMin();
}
float Scan::DistanceMax() {
  return Channel::DistanceMax();
}
float Scan::FoV() {
  return 0.0f;
}

// Channel
float Channel::DistanceMax() {
  return 0.0f;
}
float Channel::DistanceMin() {
  return 0.0f;
}
} // namespace model_x
} // namespace ce30_drivers
