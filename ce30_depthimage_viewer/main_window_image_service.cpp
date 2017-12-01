#include "main_window_image_service.h"
#include "config.h"
#include <cassert>
#include <QThread>
#include <QMessageBox>
#include "window_utils.h"

MainWindowImageService::MainWindowImageService(QWidget* parent)
  : MainWindowImageControl(parent),
    start_push_button_start_text_("Start"),
    start_push_button_stop_text_("Stop"),
    server_started_(false)
{
  server_.reset(new ImageServer(GetImageManager()));
  device_manager_.reset(new DeviceManager);
  auto ok = server_->SetDeviceManager(device_manager_);
  assert(ok);

  device_info_label_text_default_ = this->windowTitle();
}

MainWindowImageService::~MainWindowImageService() {
  server_->Stop();
}

void MainWindowImageService::OnStartPushButtonClicked() {
  MainWindowImageControl::OnStartPushButtonClicked();
  if (server_started_) {
    StopService();
  } else {
    StartService();
  }
}

void MainWindowImageService::StartService() {
  server_->SetIP(gConfig.GetIP().toStdString());
  server_->SetPort(gConfig.GetPort().toShort());
  server_->Start();
  if (!server_->Wait()) {
    QMessageBox::warning(
        this, WindowUtils::WarningMessageBoxTitle(),
        "Fail to Connect CE30!", QMessageBox::Abort);
    return;
  }
  StartPushButton()->setText(start_push_button_stop_text_);
  SettingsPushButton()->setDisabled(true);
  this->setWindowTitle(
      DeviceInfoLabelText(
          device_manager_->DeviceVersion(), device_manager_->DeviceID()));
  server_started_ = true;
}

void MainWindowImageService::StopService() {
  server_->Stop();
  StartPushButton()->setText(start_push_button_start_text_);
  SettingsPushButton()->setDisabled(false);
  this->setWindowTitle(DeviceInfoLabelTextDefault());
  server_started_ = false;
}

QString MainWindowImageService::DeviceInfoLabelText(
    const QString& version, const int& id) {
  QString text = device_info_label_text_default_ + " - Device Version: ";
  if (version.isEmpty()) {
    text += "Unknown";
  } else {
    text += version;
  }
  text += "  ";
  text += "Device ID: ";
  if (id < 0) {
    text += "Unknown";
  } else {
    text += QString::number(id);
  }
  return text;
}

QString MainWindowImageService::DeviceInfoLabelTextDefault() {
  return device_info_label_text_default_;
}
