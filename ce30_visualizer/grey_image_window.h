#ifndef GREY_IMAGEWINDOW_H
#define GREY_IMAGEWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <memory>
#include <QImage>

namespace Ui {
  class GreyImageWindow;
}

class GreyImageWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit GreyImageWindow(QWidget *parent = 0);
  ~GreyImageWindow();
public slots:
  void OnUpdateImage(std::shared_ptr<QImage> image);
protected:
  void resizeEvent(QResizeEvent *event);
  void showEvent(QShowEvent *event);
private:
  void AdjustGreyImageSizeByRatio(QLabel* label);
  Ui::GreyImageWindow *ui;
};

#endif // GREY_IMAGEWINDOW_H
