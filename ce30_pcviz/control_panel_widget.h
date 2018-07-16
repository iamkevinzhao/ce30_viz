#ifndef CONTROL_PANEL_WIDGET_H
#define CONTROL_PANEL_WIDGET_H

#include "control_panel_base.h"
#include <QWidget>

namespace Ui {
  class ControlPanelWidget;
}

namespace ce30_pcviz {
class API ControlPanelWidget : public ControlPanelBase
{
  Q_OBJECT

public:
  explicit ControlPanelWidget(QWidget *parent = 0);
  ~ControlPanelWidget();

private:
  Ui::ControlPanelWidget *ui;
};
} // namespace ce30_pcviz

#endif // CONTROL_PANEL_WIDGET_H
