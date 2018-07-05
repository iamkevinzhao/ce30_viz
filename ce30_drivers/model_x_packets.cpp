#include "model_x_packets.h"
#ifndef M_PI
#define M_PI 3.1415926
#endif

namespace ce30_drivers {
namespace model_x {
// Packet
Packet::Packet() {
  data.resize(TotalLength(), 0);
}

Channel Packet::ParseChannel(
    const uint32_t &A, const uint32_t &B, const uint16_t &t) {
  Channel channel;
  channel.x = 140.0 * std::sin(2 * M_PI * 0.005 * t);
  channel.y =
      (t < 500) ?
          ((2.0 * 140.0 / 500.0) * t + 140.0) :
          ((2.0 * 140.0 / (5000.0 - 500.0)) * t - 170);
  channel.distance = 1.0 * A / B * 1000.0 * 3.0 * 1e8 / 2.0;
  return channel;
}

std::unique_ptr<ParsedPacket> Packet::Parse() {
  if (data.size() != TotalLength()) {
    return nullptr;
  }
  if (!DetectHead(data[0], data[1])) {
    return nullptr;
  }
  uint16_t t;
  memcpy(&t, &data[2], 2);
  int index = 4;
  const int kSizeOfDim = SizeOfDim();
  const int kSizeOfPacket = data.size();
  std::unique_ptr<ParsedPacket> parsed(new ParsedPacket);
  parsed->channels.reserve(NumChannel());
  uint32_t A, B;
  while (index < kSizeOfPacket) {
    memcpy(&A, &data[index], kSizeOfDim);
    index += kSizeOfDim;
    memcpy(&B, &data[index], kSizeOfDim);
    index += kSizeOfDim;
    parsed->channels.push_back(ParseChannel(A, B, t));
    ++t;
  }
  parsed->t = t;
  return parsed;
}

// 1024
int Packet::TotalLength() {
  return 1204;
}

int Packet::SizeOfHead() {
  return 2;
}

int Packet::SizeOfT() {
  return 2;
}

int Packet::SizeOfDim() {
  return 4;
}

int Packet::SizeOfChannel() {
  return 2 * SizeOfDim();
}

int Packet::NumChannel() {
  return (TotalLength() - SizeOfHead() - SizeOfT()) / SizeOfChannel();
}

bool Packet::DetectHead(const unsigned char &alpha, const unsigned char &beta) {
  if (alpha != 0xFD) {
    return false;
  }
  if (beta != 0xFE) {
    return false;
  }
  return true;
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
