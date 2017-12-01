#include "main_window_image_control.h"
#include "config.h"
#include <QDebug>

using namespace std;

MainWindowImageControl::MainWindowImageControl(QWidget* parent)
  : MainWindow(parent)
{
  full_image_.reset(new ImageManager);
  full_image_->SetGrayImage(FullImageDefault());

  FullImageLabel()->setPixmap(*full_image_->GetPixmap());

  UpdateImagesDisplay();

  loop_timer_ = startTimer(10);
}

void MainWindowImageControl::timerEvent(QTimerEvent *event) {
  MainWindow::timerEvent(event);
  if (event->timerId() == loop_timer_) {
    UpdateImagesDisplay();
  }
}

void MainWindowImageControl::UpdateImagesDisplay() {
  auto full_image = full_image_->GetGrayImage();
  FullImageLabel()->setPixmap(QPixmap::fromImage(*full_image));
}

unique_ptr<GrayImage> MainWindowImageControl::FullImageDefault() {
  return unique_ptr<GrayImage>(new GrayImage(gConfig.FullImageSize()));
}

unique_ptr<GrayImage> MainWindowImageControl::CutFromFullImage(
    const GrayImage& image, float percetage) {
  auto result = image.Chop(GetFullImageSegmentFrameRect(percetage));
  Q_ASSERT(result);
  return result;
}

unique_ptr<GrayImage> MainWindowImageControl::HighlightFullImageSegment(
    const GrayImage& image, float percetage) {
  auto result = image.Highlight(GetFullImageSegmentFrameRect(percetage));
  Q_ASSERT(result);
  return result;
}

QRect MainWindowImageControl::GetFullImageSegmentFrameRect(
    const float& percetage) {
  int center = percetage * gConfig.FullImageSize().width();
  int half_width = gConfig.SegmentImageSize().width() / 2;
  int left = center - half_width + 1;
  int right = center + half_width - 1;
  return
      QRect(QPoint(left, 0), QPoint(right, gConfig.FullImageSize().height() - 1));
}

shared_ptr<ImageManager> MainWindowImageControl::GetImageManager() {
  return full_image_;
}
