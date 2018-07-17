#ifndef CONTROL_PANEL_WIDGET_H
#define CONTROL_PANEL_WIDGET_H

#include <QWidget>
#include "operation_handler.h"
#include "export.h"
#include <unordered_map>
#include <string>
#include <QLayout>

namespace Ui {
  class ControlPanelWidget;
}

namespace ce30_pcviz {
class API ControlPanelWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ControlPanelWidget(QWidget *parent = 0);
  ~ControlPanelWidget();
  static void ClearGridLayout(QLayout* layout);

public slots:
  void OnShow(std::vector<ce30_pcviz::CtrlShortcut> shortcuts);

private:
  Ui::ControlPanelWidget *ui;
  std::vector<CtrlShortcut> ctrl_shortcut_map_;
};
} // namespace ce30_pcviz

#endif // CONTROL_PANEL_WIDGET_H
