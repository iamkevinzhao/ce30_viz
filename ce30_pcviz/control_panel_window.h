#ifndef CONTROL_PANEL_WINDOW_H
#define CONTROL_PANEL_WINDOW_H

#include <QMainWindow>
#include "operation_handler.h"
#include "export.h"
#include <unordered_map>
#include <string>
#include <QLayout>

namespace Ui {
  class ControlPanelWindow;
}

namespace ce30_pcviz {
class API ControlPanelWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit ControlPanelWindow(QWidget *parent = 0);
  ~ControlPanelWindow();
  static void ClearGridLayout(QLayout* layout);

public slots:
  void OnShow(std::vector<ce30_pcviz::CtrlShortcut> shortcuts);

private:
  Ui::ControlPanelWindow *ui;
  std::vector<CtrlShortcut> ctrl_shortcut_map_;
};
} // namespace ce30_pcviz

#endif // CONTROL_PANEL_WINDOW_H
