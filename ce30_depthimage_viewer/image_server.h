#ifndef IMAGE_SERVER_H
#define IMAGE_SERVER_H

#include "image_manager.h"
#include "device_manager.h"
#include <thread>
#include <condition_variable>
#include "config.h"

class ImageServer
{
public:
  ImageServer(std::shared_ptr<ImageManager> manager);
  bool Start();
  bool Stop();
  bool Wait();
  void SetIP(const std::string& ip);
  void SetPort(const uint16_t& port);
  bool SetDeviceManager(std::shared_ptr<DeviceManager> manager);
private:
  void Execute(Config config);

  std::shared_ptr<ImageManager> image_manager_;
  std::shared_ptr<DeviceManager> device_manager_;
  std::thread thread_;

  class Signal {
   public:
    void Set(const bool& signal);
    bool Get();
   private:
    std::mutex mutex_;
    bool signal_;
  };
  Signal stop_signal_;
  Signal connection_signal_;

  std::mutex connection_mutex_;
  std::condition_variable connection_condition_;

  std::string ip_;
  uint16_t port_;
};

#endif // IMAGE_SERVER_H
