#ifndef MODEL_X_PACKETS_H
#define MODEL_X_PACKETS_H

#include "packets.h"

namespace ce30_drivers {
namespace model_x {
struct CE30_DRIVERS_API Channel : public ce30_drivers::Channel {
  static float DistanceMax();
  static float DistanceMin();
  int x;
  int y;
};

struct CE30_DRIVERS_API ParsedPacket : public ce30_drivers::ParsedPacket {
  uint16_t t;
  std::vector<Channel> channels;
};

struct CE30_DRIVERS_API Packet : public ce30_drivers::Packet {
  Packet();
  std::unique_ptr<ParsedPacket> Parse();
  inline static int TotalLength();
  inline static int SizeOfHead();
  inline static int SizeOfT();
  inline static int SizeOfDim();
  inline static int SizeOfChannel();
  inline static int NumChannel();
  static bool DetectHead(const unsigned char& alpha, const unsigned char& beta);
  static Channel ParseChannel(const uint32_t& A, const uint32_t& B, const uint16_t& t);
};

class CE30_DRIVERS_API Scan : public ce30_drivers::Scan {
 public:
  Scan();
  void AddFromPacket(const ParsedPacket& packet);
  bool Ready() override;
  void Reset() override;
  Channel at(const int& x, const int& y) const;
  static int Width();
  static int Height();
  static float DistanceMin();
  static float DistanceMax();
  static float FoV();
};
} // namespace model_x
} // namespace ce30_drivers

#endif // MODEL_X_PACKETS_H
