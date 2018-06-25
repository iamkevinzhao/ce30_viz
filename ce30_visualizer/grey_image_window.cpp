#include "grey_image_window.h"
#include "ui_grey_image_window.h"
#include <QPixmap>
#include <ce30_pcviz/gray_image.h>
#include <ce30_drivers/ce30_d_driver.h>

const float kGreyImageWidthHeightRatio =
    1.0f * ce30_d::Scan::Width() / ce30_d::Scan::Height();

GreyImageWindow::GreyImageWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GreyImageWindow)
{
  ui->setupUi(this);
  ui->GreyImageDisplayLabel->setScaledContents(true);
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
