#ifndef MAIN_WINDOW_POINT_CLOUD_SERVICE_H
#define MAIN_WINDOW_POINT_CLOUD_SERVICE_H

#include "main_window_image_service.h"
#include <memory>
#include "point_cloud_viewer.h"

class MainWindowPointCloudService : public MainWindowImageService
{
public:
  MainWindowPointCloudService(QWidget* parent = 0);
  virtual ~MainWindowPointCloudService();
protected:
  virtual void OnPointCloudPushButtonClicked();
private:
  std::unique_ptr<PointCloudViewer> pcviewer_;
};

#endif // MAIN_WINDOW_POINT_CLOUD_SERVICE_H
