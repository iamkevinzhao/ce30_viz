#include "grey_image_window.h"
#include "ui_grey_image_window.h"
#include <QPixmap>
#include "grey_image.h"
#include <ce30_driver/packet.h>

const float kGreyImageWidthHeightRatio = 1.0f * ce30_driver::Scan::Width() / ce30_driver::Scan::Height();

GreyImageWindow::GreyImageWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GreyImageWindow)
{
  ui->setupUi(this);
//  GrayImage image(320, 20, 0, 100);
//  for (int x = 0; x < 320; ++x) {
//    for (int y = 0; y < 20; ++y) {
//      image.SetPixel(QPoint(x, y), 50);
//    }
//  }
  ui->GreyImageDisplayLabel->setScaledContents(true);
//  ui->GreyImageDisplayLabel->setPixmap(QPixmap::fromImage(image));
  this->resize(800, 200);
}

GreyImageWindow::~GreyImageWindow()
{
  delete ui;
}

void GreyImageWindow::resizeEvent(QResizeEvent *event) {
  AdjustGreyImageSizeByRatio(ui->GreyImageDisplayLabel);
}

void GreyImageWindow::showEvent(QShowEvent *event) {
  AdjustGreyImageSizeByRatio(ui->GreyImageDisplayLabel);
}

void GreyImageWindow::OnUpdateImage(std::shared_ptr<QImage> image) {
  if (!image) {
    return;
  }
  ui->GreyImageDisplayLabel->setPixmap(QPixmap::fromImage(*image));
}

void GreyImageWindow::AdjustGreyImageSizeByRatio(QLabel* label) {
  label->setFixedHeight(label->width() / kGreyImageWidthHeightRatio);
}
