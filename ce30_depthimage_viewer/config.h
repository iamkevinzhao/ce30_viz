#ifndef CONFIG_H
#define CONFIG_H

#include <QSize>
#include <QString>
#include <QSettings>

class Config
{
public:
  Config();
  ~Config();
  inline QSize FullImageSize() const {
    return full_image_size_;
  }
  inline QSize SegmentImageSize() const {
    return segment_image_size_;
  }
  inline float FullImageHVRatio() const {
    return full_image_hv_ratio_;
  }
  inline float SegmentImageHVRatio() const {
    return segment_image_hv_ratio_;
  }
  inline QString GetIP() const {
    return ip_;
  }
  inline void SetIP(const QString& ip) {
    ip_ = ip;
  }
  inline QString GetPort() const {
    return port_;
  }
  inline void SetPort(const QString& port) {
    port_ = port;
  }
  inline QString GetIniFilePath() const {
    return ini_file_path_;
  }
  inline void SetDeviceID(const int& id) {
    device_id_ = id;
  }
  inline int GetDeviceID() const {
    return device_id_;
  }
  inline void SetDeviceInvalidID() {
    device_id_ = -1;
  }
  inline bool HoldValidDeviceID() {
    return device_id_ >= 0;
  }
  void Load(const QSettings& settings);
  void Save(QSettings& settings);
  void Default();
private:
  // Image Display
  float full_image_hv_ratio_;
  float segment_image_hv_ratio_;
  QSize segment_image_size_;
  QSize full_image_size_;

  // Ini File
  QString ini_file_path_;

  // Network
  QString network_group_;
  QString ip_default_;
  QString port_default_;
  QString ip_;
  QString ip_key_;
  QString port_;
  QString port_key_;

  // Device
  int device_id_;
};

extern Config gConfig;
#endif // CONFIG_H
