#ifndef GRAY_IMAGE_H
#define GRAY_IMAGE_H

#include <QImage>
#include <memory>
#include <QDebug>

class GrayImage : public QImage
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

#endif // GRAY_IMAGE_H
