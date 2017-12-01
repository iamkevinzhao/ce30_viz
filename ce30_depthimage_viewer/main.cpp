#include <QApplication>
#include "main_window_point_cloud_service.h"
#include "config.h"

void LoadSettings() {
  QSettings settings(gConfig.GetIniFilePath(), QSettings::IniFormat);
  gConfig.Load(settings);

}

void SaveSettings() {
  QSettings settings(gConfig.GetIniFilePath(), QSettings::IniFormat);
  gConfig.Save(settings);
}

int main(int argc, char* argv[])
{
  LoadSettings();
  QApplication app(argc, argv);
  MainWindowPointCloudService win;
  win.show();
  auto rt = app.exec();
  SaveSettings();
  return rt;
}
