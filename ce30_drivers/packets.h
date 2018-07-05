#ifndef CE30_DRIVERS_PACKETS_H_
#define CE30_DRIVERS_PACKETS_H_

#include "ce30_drivers_export.h"
#include <chrono>
#include <vector>
#include <memory>
#include "ce30_drivers/data_types.h"

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

struct CE30_DRIVERS_API Channel {
  virtual ~Channel();
  float distance;
  float amplitude;
  virtual Point point() const;
};

struct CE30_DRIVERS_API Column {
  virtual ~Column();
  // std::vector<Channel> channels;
};

struct CE30_DRIVERS_API ParsedPacket {
  virtual ~ParsedPacket();
  // std::vector<Column> columns;
};

struct CE30_DRIVERS_API Packet : ce30_drivers::PacketBase {
  virtual ~Packet();
  virtual std::unique_ptr<ParsedPacket> ParseBase();
};

class CE30_DRIVERS_API Scan {
 public:
  virtual ~Scan();
  virtual bool Ready();
  virtual void Reset();
  virtual int Width() const;
  virtual int Height() const;
};
} // namespace ce30_drivers

#endif // CE30_DRIVERS_PACKETS_H_
