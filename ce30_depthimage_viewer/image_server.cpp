#include "image_server.h"
#include <QDebug>
#include <QThread>
#include <ce30_driver/udp_socket.h>
#include <fstream>
#include <ce30_driver/utils.h>
#include <iostream>
#include "config.h"

using namespace std;
using namespace ce30_driver;

void ImageServer::Signal::Set(const bool &signal) {
  unique_lock<mutex> lock(mutex_);
  signal_ = signal;
  lock.unlock();
}

bool ImageServer::Signal::Get() {
  unique_lock<mutex> lock(mutex_);
  auto signal = signal_;
  lock.unlock();
  return signal;
}

ImageServer::ImageServer(std::shared_ptr<ImageManager> manager)
  : image_manager_(manager)
{
  stop_signal_.Set(true);
}

bool ImageServer::SetDeviceManager(std::shared_ptr<DeviceManager> manager) {
  if (stop_signal_.Get()) {
    device_manager_ = manager;
    return true;
  } else {
    return false;
  }
}

void ImageServer::SetIP(const string& ip) {
  ip_ = ip;
}

void ImageServer::SetPort(const uint16_t& port) {
  port_ = port;
}

bool ImageServer::Start() {
  if (!image_manager_) {
    return false;
  }
  stop_signal_.Set(false);
  connection_signal_.Set(false);
  auto config = gConfig;
  if (gConfig.HoldValidDeviceID()) {
    gConfig.SetDeviceInvalidID();
  }
  thread_ = std::thread(&ImageServer::Execute, this, config);
  return true;
}

bool ImageServer::Stop() {
  stop_signal_.Set(true);
  if (thread_.joinable()) {
    thread_.join();
  }
  return true;
}

void ImageServer::Execute(Config config) {
  auto HandleConnectFailure = [&](){
    connection_signal_.Set(false);
    connection_condition_.notify_all();
  };

  UDPSocket socket(ip_, port_);
  if (socket.Connect() != Diagnose::connect_successful) {
    HandleConnectFailure();
    return;
  }

  string version = "";
  device_manager_->SetDeviceVersion("");
  if (GetVersion(version, socket)) {
    device_manager_->SetDeviceVersion(QString::fromStdString(version));
  } else {
    HandleConnectFailure();
    return;
  }

//  if (config.HoldValidDeviceID()) {
//    if (!SetDeviceID(config.GetDeviceID(), socket)) {
//      HandleConnectFailure();
//      return;
//    }
//  }

//  int id = -1;
//  device_manager_->SetDeviceID(id);
//  if (GetDeviceID(id, socket)) {
//    device_manager_->SetDeviceID(id);
//  } else {
//    HandleConnectFailure();
//    return;
//  }

  if (!StartRunning(socket)) {
    HandleConnectFailure();
    return;
  }

  connection_signal_.Set(true);
  connection_condition_.notify_all();

  Scan scan;
  while (!stop_signal_.Get()) {
    Packet packet;
    if (socket.GetPacket(packet, 10) != Diagnose::receive_successful) {
      continue;
    }
    if (!scan.Ready()) {
      auto parsed_packet = packet.Parse();
      if (parsed_packet) {
        scan.AddColumnsFromPacket(*parsed_packet);
      }
    }
    if (scan.Ready()) {
      image_manager_->SetGrayImageFromScan(scan);
      scan.Reset();
    }
  }

  StopRunning(socket);
}

bool ImageServer::Wait() {
  unique_lock<mutex> lock(connection_mutex_);
  connection_condition_.wait(lock);
  bool result = connection_signal_.Get();
  if (!result) {
    Stop();
  }
  return result;
}
