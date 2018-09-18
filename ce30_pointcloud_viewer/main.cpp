#include <QApplication>
#include "point_cloud_viewer.h"
#include <QTimer>
#include "grey_image_window.h"
#include <QObject>

#ifdef FAKE_POINTCLOUD
#include "fake_point_cloud_viewer.h"
#endif

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
#ifdef FAKE_POINTCLOUD
  FakePointCloudViewer viewer;
#else
  PointCloudViewer viewer;
#endif
  GreyImageWindow win;
  win.show();

  QObject::connect(
      &viewer, SIGNAL(UpdateImage(std::shared_ptr<QImage>)),
      &win, SLOT(OnUpdateImage(std::shared_ptr<QImage>)));
  QObject::connect(
      &viewer, SIGNAL(ShowGrayImage()),
      &win, SLOT(show()));
  QObject::connect(
      &viewer, SIGNAL(HideGrayImage()),
      &win, SLOT(hide()));
  return app.exec();
}
