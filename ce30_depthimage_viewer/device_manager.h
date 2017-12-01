#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <mutex>
#include <QString>

class DeviceManager
{
public:
  DeviceManager();
  QString DeviceVersion();
  void SetDeviceVersion(const QString& version);
  void SetDeviceID(const int& id);
  int DeviceID();
private:
  std::mutex version_mutex_;
  QString version_;

  std::mutex id_mutex_;
  int id_;
};

#endif // DEVICE_MANAGER_H
