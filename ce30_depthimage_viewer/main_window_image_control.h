#ifndef MAIN_WINDOW_IMAGE_CONTROL_H
#define MAIN_WINDOW_IMAGE_CONTROL_H

#include "main_window.h"
#include <QWidget>
#include <QTimerEvent>
#include <memory>
#include "image_manager.h"
#include "gray_image.h"

class MainWindowImageControl : public MainWindow
{
public:
  MainWindowImageControl(QWidget* parent = 0);
protected:
  void timerEvent(QTimerEvent *event);
  std::shared_ptr<ImageManager> GetImageManager();
private:
  std::unique_ptr<GrayImage> FullImageDefault();
  QRect GetFullImageSegmentFrameRect(const float& percetage);
  std::unique_ptr<GrayImage> CutFromFullImage(
      const GrayImage& image, float percetage);
  std::unique_ptr<GrayImage> HighlightFullImageSegment(
      const GrayImage& image, float percetage);
  void UpdateImagesDisplay();

  std::shared_ptr<ImageManager> full_image_;
  int loop_timer_;
};

#endif // MAIN_WINDOW_IMAGE_CONTROL_H
