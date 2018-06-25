#ifndef PACKET_H
#define PACKET_H

#include <vector>
#include <chrono>
#include <memory>
#include <unordered_map>
#include "ce30_drivers_export.h"
#include "data_types.h"
#include "packets.h"

namespace ce30_drivers {
namespace model_d {
/**
 * @brief contains the data collected by a photosensitive cell
 */
struct CE30_DRIVERS_API Channel : ce30_drivers::Channel {
  /**
   * @brief constructor
   */
  Channel();

  Point point() const override;

  unsigned short grey_value;
  /**
   * @brief maximum distance
   * @return distance in meters
   */
  static float DistanceMax();
  /**
   * @brief minimum distance
   * @return distance in meters
   */
  static float DistanceMin();

  static unsigned short GreyValueMax();

  static unsigned short GreyValueMin();
  /**
   * @brief horizontal azimuth in degrees
   */
  float h_azimuth;
  /**
   * @brief vertical azimuth in degrees
   */
  float v_azimuth;
};

/**
 * @brief a collection of channels which are vertically aligned
 */
struct CE30_DRIVERS_API Column : ce30_drivers::Column {
  /**
   * @brief constructor
   */
  Column();

  std::vector<Channel> channels;
  /**
   * @brief azimuth of this column
   */
  float azimuth;
  /**
   * @brief the number of channels in a column
   * @return the number of channels
   */
  static int ChannelNum();
};

/**
 * @brief contains data after being parsed from a packet
 */
struct CE30_DRIVERS_API ParsedPacket : ce30_drivers::ParsedPacket {
  /**
   * @brief constructor
   */
  ParsedPacket();

  std::vector<Column> columns;
  /**
   * @brief time stamp parsed from packet
   */
  double time_stamp;
  /**
   * @brief the number of columns in a packet
   * @return the number of columns
   */
  static int ColumnNum();

  bool grey_image;
};

/**
 * @brief a scan of channels which contains data from all channels
 */
class CE30_DRIVERS_API Scan {
public:
  /**
   * @brief constructor
   */
  Scan();
  /**
   * @brief add a column to this scan
   * @param column the column to add
   */
  void AddColumn(const Column& column);
  /**
   * @brief add columns from a parsed packet
   * @param packet the parsed packet to add
   */
  void AddColumnsFromPacket(const ParsedPacket& packet);
  /**
   * @brief check whether a scan is ready
   * @return true if the scan is ready; otherwise, false
   */
  bool Ready();
  /**
   * @brief clear all columns in this scan
   */
  void Reset();
  /**
   * @brief get the channel by giving its 2D indices in the scan
   * @param x horizontal index
   * @param y vertical index
   * @return the channel
   */
  Channel at(const int& x, const int& y) const;
  /**
   * @brief the width of the scan
   * @return the number of channels in horizontal
   */
  static int Width();
  /**
   * @brief the height of the scan
   * @return the number of channels in vertical
   */
  static int Height();
  /**
   * @brief refer to \ref Channel::DistanceMin()
   */
  static float DistanceMin();
  /**
   * @brief refer to \ref Channel::DistanceMax()
   */
  static float DistanceMax();
  /**
   * @brief the number of columns in a scan
   * @return the number of columns
   */
  static int ColumnNum();
  /**
   * @brief enumerate all azimuth values
   * @return the vector of azimuths in degrees
   */
  static std::vector<float> AzimuthMap();
  /**
   * @brief field of view
   * @return FoV in degree
   */
  static float FoV();
  static float AzimuthDelta();
  static int WhichColumn(const float& azimuth);
  /**
   * @brief given vertical index, find the vertical azimuth
   * @param i the vertical index
   * @return the vertical azimuth in degrees
   */
  inline static float LookUpVerticalAzimuth(const int& i);
private:
  std::unordered_map<int, Column> columns_;
};

/**
 * @brief device measurement data packet
 */
struct CE30_DRIVERS_API Packet : public ce30_drivers::Packet {
  /**
   * @brief constructor
   */
  Packet();
  /**
   * @brief parse this packet
   * @return the pointer to the parsed packet
   */
  std::unique_ptr<ParsedPacket> Parse();

  std::unique_ptr<ce30_drivers::ParsedPacket> ParseBase() override;
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int HeaderBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int ColumnIdentifierBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int AzimuthBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int DistanceBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int AmplitudeBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int TimeStampBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static int FactoryBytes();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static unsigned char ColumnIdentifierHigh();
  /// @endcond
  /// @cond DO_NOT_DOCUMENT_THIS
  inline static unsigned char ColumnIdentifierLow();
  /// @endcond
private:
  inline static int GreyImageStatusIndex();
  static bool IsGreyImage(const char& grey_image_byte);
  float ParseAzimuth(const unsigned char& high, const unsigned char& low);
  float ParseDistance(const unsigned char& high, const unsigned char& low);
  unsigned short ParseGreyValue(const unsigned char& high, const unsigned char& low);
  float ParseAmplitude(const unsigned char& raw);
  double ParseTimeStamp(std::vector<unsigned char> stamp_raw);
};

/**
 * @brief the base class of all request types
 */
struct CE30_DRIVERS_API RequestPacket : public PacketBase {
  /**
   * @brief constructor
   */
  RequestPacket();
  virtual ~RequestPacket();
  /**
   * @brief set command string
   * @param cmd the command string
   * @return true if the command string is valid; otherwise, false
   */
  bool SetCmdString(const std::string& cmd);
  /**
   * @brief reset this packet
   */
  void ResetPacket();
};

/**
 * @brief packet of command for requesting device version
 */
struct CE30_DRIVERS_API VersionRequestPacket : public RequestPacket {
  /**
   * @brief constructor
   */
  VersionRequestPacket();
};

/**
 * @brief packet of version information sent by the device
 */
struct CE30_DRIVERS_API VersionResponsePacket : public PacketBase {
  VersionResponsePacket();
  /**
   * @brief parse the packet for the version string
   * @return the version string
   */
  std::string GetVersionString() const;
};

/**
 * @brief packet of command for setting device ID
 */
struct CE30_DRIVERS_API SetIDRequestPacket : public RequestPacket {
  /**
   * @brief constructor
   * @param id the ID to set
   */
  SetIDRequestPacket(const int32_t& id);
};

/**
 * @brief meta class of common response packets
 */
struct CE30_DRIVERS_API CommonResponsePacket : public PacketBase {
  /**
   * @brief constructor
   */
  CommonResponsePacket();
  /**
   * @brief if the device gave a positive response to the request
   * @return true if positive; otherwise, false
   */
  bool Successful() const;
};

using SetIDResponsePacket = CommonResponsePacket;

/**
 * @brief packet of command for acquiring device ID
 */
struct CE30_DRIVERS_API GetIDRequestPacket : public RequestPacket {
  /**
   * @brief constructor
   */
  GetIDRequestPacket();
};

/**
 * @brief response packet to the ID requeset
 */
struct CE30_DRIVERS_API GetIDResponsePacket : public PacketBase {
  /**
   * @brief constructor
   */
  GetIDResponsePacket();
  /**
   * @brief parse the packet for ID
   * @return the ID
   */
  int32_t ID() const;
};

/// @cond DO_NOT_DOCUMENT_THIS
struct CE30_DRIVERS_API StampSyncRequestPacket : public RequestPacket {
  StampSyncRequestPacket(const uint32_t& microseconds);
};
/// @endcond

// Identical Functionality
using StampSyncResponsePacket = CommonResponsePacket;

/**
 * @brief packet of command for starting measuring
 */
struct CE30_DRIVERS_API StartRequestPacket : public RequestPacket {
  /**
   * @brief constructor
   */
  StartRequestPacket();
};

/**
 * @brief packet of command for stopping measuring
 */
struct CE30_DRIVERS_API StopRequestPacket : public RequestPacket {
  /**
   * @brief constructor
   */
  StopRequestPacket();
};

/// @cond DO_NOT_DOCUMENT_THIS
struct CE30_DRIVERS_API EnableFilterRequestPacket : public RequestPacket {
  EnableFilterRequestPacket();
};
/// @endcond

using EnableFilterResponsePacket = CommonResponsePacket;

/// @cond DO_NOT_DOCUMENT_THIS
struct CE30_DRIVERS_API DisableFilterRequestPacket : public RequestPacket {
  DisableFilterRequestPacket();
};
/// @endcond

using DisableFilterResponsePacket = CommonResponsePacket;
} // namespace model_d
} // namespace ce30_drivers

#endif // PACKET_H
