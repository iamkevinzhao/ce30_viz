#include "model_x_packets.h"
#ifndef M_PI
#define M_PI 3.1415926
#endif

#include <iostream>
#include <cmath>
#include <string.h>

namespace ce30_drivers {
namespace model_x {
// Packet
Packet::Packet() {
  data.resize(TotalLength(), 0);
}

Channel Packet::ParseChannel(
    const uint32_t &A, const uint32_t &B, int t) {
  //std::cout << t <<std::endl;
  //t = t - 106 - 1000;
  t = t - 6 + 4800;

  if (t >= 5000) {
    t = t % 4999;
  }

  Channel channel;
  if (t <= 140) {
    channel.x = 0;
    channel.y = 0;
    channel.distance = 0;
    return channel;
  }
  static int amp = 140;
  static int b = 140;
  static int ampy = 70;
  static double k1 = -(2.0 * ampy) / b;
  static double k2 = (2.0 * ampy) / (5000.0 - b);


  double dist_sortx = -(1.0 * amp * std::sin(2.0 * M_PI * 0.005 * t));
  double data_distxp = 26214.0 + (dist_sortx / 200.0) * 32767.0;
  double data_distxn = 26214.0 - (dist_sortx / 200.0) * 32767.0;
  channel.x = ((data_distxp - data_distxn) / 22936.0) * 5.0;
  channel.x = -channel.x;

  double dist_sorty = (t <= b) ? (k1 * t + ampy) : (k2 * t - 74.03);
  // std::cout << dist_sorty << " " << t << " " << b << " " << " " << k1 << " " << t << " " << ampy << " " << k2 << std::endl;
  double data_distyn = 26214.0 + (dist_sorty / 200.0) * 32767.0;
  double data_distyp = 26214.0 - (dist_sorty / 200.0) * 32767.0;
  // std::cout << dist_sorty << " " << data_distyp << " "  << data_distyn << " " << (data_distyp - data_distyn) << std::endl;
  channel.y = ((data_distyp - data_distyn) / 22936.0) * 6.0 + 2.0;
  channel.y += 4.0f;

  channel.distance = 1.0 * A / B * 1000.0 * 3.0 / 10.0 / 2.0;
  // if (channel.distance > 25.0 && channel.distance < 30.0) {
    // std::cout <<channel.x <<" " <<channel.y <<std::endl;
  //}
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
  uint8_t buf2[2];
  buf2[0] = data[3];
  buf2[1] = data[2];
  memcpy(&t, buf2, 2);
  int index = 4;
  const int kSizeOfDim = SizeOfDim();
  const int kSizeOfPacket = data.size();
  std::unique_ptr<ParsedPacket> parsed(new ParsedPacket);
  parsed->channels.reserve(NumChannel());
  uint32_t A, B;
  uint8_t buf4[4];
  int channel_num = 0;
  while (index < kSizeOfPacket) {
    ++channel_num;
//    if (channel_num > 149) {
//      continue;
//    }
//    if (t >= 0 && t <= 140) {
//      continue;
//    }
    buf4[0] = data[index + 3];
    buf4[1] = data[index + 2];
    buf4[2] = data[index + 1];
    buf4[3] = data[index];
    memcpy(&A, buf4, kSizeOfDim);
    index += kSizeOfDim;

    buf4[0] = data[index + 3];
    buf4[1] = data[index + 2];
    buf4[2] = data[index + 1];
    buf4[3] = data[index];
    memcpy(&B, buf4, kSizeOfDim);
    index += kSizeOfDim;
    if (channel_num <= 149) {
      parsed->channels.push_back(ParseChannel(A, B, t));
    }
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

int Packet::TMax() {
  return 5000;
}

int Packet::TMin() {
  return 0;
}

int Packet::NumT() {
  return TMax() - TMin() + 1;
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

void Scan::AddFromPacket(const ParsedPacket& packet) {
  packets_[packet.t] = packet;
}

bool Scan::Ready() {
  return packets_.size() > std::floor(1.0f * Packet::NumT() / Packet::NumChannel());
}

void Scan::Reset() {
  packets_.clear();
}

std::vector<Channel> Scan::GetChannels() const {
  std::vector<Channel> channels;
  channels.reserve(packets_.size() * Packet::NumChannel());
  for (auto& packet : packets_) {
    channels.insert(channels.end(), packet.second.channels.begin(), packet.second.channels.end());
  }
  return channels;
}

float Scan::DistanceMin() {
  return Channel::DistanceMin();
}

float Scan::DistanceMax() {
  return Channel::DistanceMax();
}

// Channel
float Channel::DistanceMax() {
  return 0.0f;
}
float Channel::DistanceMin() {
  return 0.0f;
}

LaserOnSendPacket::LaserOnSendPacket() {
  std::string cmd = {0x30, 0x30, 0x30, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
  data.resize(cmd.size(), 0);
  memcpy(data.data(), cmd.c_str(), cmd.size());
}

LaserOffSendPacket::LaserOffSendPacket() {
  std::string cmd = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
  data.resize(cmd.size(), 0);
  memcpy(data.data(), cmd.c_str(), cmd.size());
}

MEMSOn::MEMSOn() {
  std::string cmd = {0x30, 0x30, 0x30, 0x33, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
  data.resize(cmd.size(), 0);
  memcpy(data.data(), cmd.c_str(), cmd.size());
}

MEMSOff::MEMSOff() {
  std::string cmd = {0x30, 0x30, 0x30, 0x32, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
  data.resize(cmd.size(), 0);
  memcpy(data.data(), cmd.c_str(), cmd.size());
}
} // namespace model_x
} // namespace ce30_drivers
