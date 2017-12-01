#ifndef MAIN_WINDOW_IMAGE_SERVICE_H
#define MAIN_WINDOW_IMAGE_SERVICE_H

#include "main_window_image_control.h"
#include "image_server.h"
#include <memory>
#include "device_manager.h"

class MainWindowImageService : public MainWindowImageControl
{
public:
  MainWindowImageService(QWidget* parent = 0);
  virtual ~MainWindowImageService();
  void OnStartPushButtonClicked();
private:
  void StartService();
  void StopService();
  QString DeviceInfoLabelText(const QString& version, const int& id);
  QString DeviceInfoLabelTextDefault();

  std::unique_ptr<ImageServer> server_;
  QString start_push_button_start_text_;
  QString start_push_button_stop_text_;
  bool server_started_;
  std::shared_ptr<DeviceManager> device_manager_;
  QString device_info_label_text_default_;
};

#endif // MAIN_WINDOW_IMAGE_SERVICE_H
