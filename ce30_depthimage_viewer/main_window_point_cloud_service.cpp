#include "main_window_point_cloud_service.h"

MainWindowPointCloudService::MainWindowPointCloudService(QWidget* parent)
  : MainWindowImageService(parent)
{
  pcviewer_.reset(new PointCloudViewer);
}

MainWindowPointCloudService::~MainWindowPointCloudService() {

}

void MainWindowPointCloudService::OnPointCloudPushButtonClicked() {
  MainWindowImageService::OnPointCloudPushButtonClicked();
  if (!pcviewer_->Displaying()) {
    pcviewer_->Display();
  }
}
