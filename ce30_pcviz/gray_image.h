#ifndef CE30_PCVIZ_GRAY_IMAGE_H_
#define CE30_PCVIZ_GRAY_IMAGE_H_

#include <QImage>
#include <memory>
#include <QDebug>
#include "export.h"

namespace ce30_pcviz {
class API GrayImage : public QImage
{
public:
  GrayImage(const GrayImage& image);
  GrayImage(const QSize& size);
  GrayImage(
      const int& width, const int& height, const float& from, const float& to);
  GrayImage(const QSize& size, const float& from, const float& to);
  inline void SetPixel(const QPoint &pt, const float& value) {
    return setPixel(pt, GetColorIndex(value));
  }
  inline int GetColorIndex(const float& value) {
    return
        (kIndexMax - kIndexMin) * (value - from_) / (to_ - from_) + kIndexMin;
  }
  std::unique_ptr<GrayImage> Chop(const QRect& region) const;
  std::unique_ptr<GrayImage> Highlight(const QRect& region) const;
private:
  QVector<QRgb> GenerateColorTable();
  bool IsRegionValid(const QRect& region) const;
  float from_;
  float to_;
  static const int kIndexMax;
  static const int kIndexMin;
  static const int kHighlightDelta;
};
} // namespace ce30_pcviz
#endif // CE30_PCVIZ_GRAY_IMAGE_H_
