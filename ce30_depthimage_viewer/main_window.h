#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QPushButton>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  virtual ~MainWindow();

protected:
  void resizeEvent(QResizeEvent* event) override;
  QLabel* FullImageLabel();
  QPushButton* StartPushButton();
  QPushButton* SettingsPushButton();
  QPushButton* PointCloudPushButton();
  virtual void OnStartPushButtonClicked();
  virtual void OnPointCloudPushButtonClicked();
private slots:
  void on_settingsPushButton_clicked();

  void on_startPushButton_clicked();

  void on_pointCloudPushButton_clicked();

private:
  Ui::MainWindow *ui;
  QString device_info_label_text_default_;
};

#endif // MAIN_WINDOW_H
