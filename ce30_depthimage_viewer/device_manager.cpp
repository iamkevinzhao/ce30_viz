#include "device_manager.h"

using namespace std;

DeviceManager::DeviceManager()
{

}

QString DeviceManager::DeviceVersion() {
  version_mutex_.lock();
  auto result = version_;
  version_mutex_.unlock();
  return result;
}

void DeviceManager::SetDeviceVersion(const QString &version) {
  version_mutex_.lock();
  version_ = version;
  version_mutex_.unlock();
}

void DeviceManager::SetDeviceID(const int &id) {
  id_mutex_.lock();
  id_ = id;
  id_mutex_.unlock();
}

int DeviceManager::DeviceID() {
  id_mutex_.lock();
  auto result = id_;
  id_mutex_.unlock();
  return id_;
}
