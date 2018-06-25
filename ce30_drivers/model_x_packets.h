#ifndef MODEL_X_PACKETS_H
#define MODEL_X_PACKETS_H

#include "packets.h"

namespace ce30_drivers {
namespace model_x {
struct CE30_DRIVERS_API Channel : public ce30_drivers::Channel {
  static float DistanceMax();
  static float DistanceMin();
};

struct CE30_DRIVERS_API ParsedPacket : public ce30_drivers::ParsedPacket {

};

struct CE30_DRIVERS_API Packet : public ce30_drivers::Packet {
  Packet();
  std::unique_ptr<ParsedPacket> Parse();
  inline static int TotalLength();
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
