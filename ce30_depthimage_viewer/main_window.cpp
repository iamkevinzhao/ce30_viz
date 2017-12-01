#include "main_window.h"
#include "ui_main_window.h"
#include <QDebug>
#include "window_utils.h"
#include "config.h"
#include "settings_dialog.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->fullImageLabel->setScaledContents(true);

  WindowUtils::MoveToCenter(this);

  this->showMaximized();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
  ui->fullImageLabel->setFixedHeight(
      ui->fullImageLabel->width() / gConfig.FullImageHVRatio());
}

QLabel* MainWindow::FullImageLabel() {
  return ui->fullImageLabel;
}

QPushButton* MainWindow::StartPushButton() {
  return ui->startPushButton;
}

QPushButton* MainWindow::SettingsPushButton() {
  return ui->settingsPushButton;
}

QPushButton* MainWindow::PointCloudPushButton() {
  return ui->pointCloudPushButton;
}

void MainWindow::OnStartPushButtonClicked() {

}

void MainWindow::OnPointCloudPushButtonClicked() {

}

void MainWindow::on_settingsPushButton_clicked()
{
  SettingsDialog dialog;
  dialog.exec();
}

void MainWindow::on_startPushButton_clicked()
{
  OnStartPushButtonClicked();
}

void MainWindow::on_pointCloudPushButton_clicked()
{
  OnPointCloudPushButtonClicked();
}
