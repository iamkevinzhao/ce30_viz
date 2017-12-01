#include "window_utils.h"
#include <QApplication>
#include <QDesktopWidget>

WindowUtils::WindowUtils()
{

}

void WindowUtils::MoveToCenter(QWidget *widget) {
  widget->move(
      QApplication::desktop()->availableGeometry().center() -
        widget->rect().center());
}

QString WindowUtils::WarningMessageBoxTitle() {
  return "Warning";
}
