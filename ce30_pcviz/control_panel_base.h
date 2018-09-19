#ifndef CONTROL_PANEL_BASE_H
#define CONTROL_PANEL_BASE_H

#include <QWidget>
#include <vector>
#include "operation_handler.h"
#include "export.h"
#include <string>
#include <QGridLayout>

namespace ce30_pcviz {
class API ControlPanelBase : public QWidget {
  Q_OBJECT
 public:
  ControlPanelBase(QWidget *parent = 0);
  virtual ~ControlPanelBase();
  static void ClearGridLayout(QLayout* layout);
  void AppendWidget(QWidget* widget);
  void AppendWidgets(const std::vector<QWidget*>& widgets);
  QWidget* Widget();
public slots:
  void OnShow(std::vector<ce30_pcviz::CtrlShortcut> shortcuts);
protected:
  void SetButtonGridLayout(QGridLayout* layout);
private:
  std::vector<CtrlShortcut> ctrl_shortcut_map_;
  QGridLayout* button_grid_layout_;
  std::vector<QWidget*> appended_widgets_;
};
} // namespace ce30_pcviz

#endif // CONTROL_PANEL_BASE_H
