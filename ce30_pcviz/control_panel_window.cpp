#include "control_panel_window.h"
#include "ui_control_panel_window.h"
#include <QPushButton>

namespace ce30_pcviz {
ControlPanelWindow::ControlPanelWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::ControlPanelWindow)
{
  ui->setupUi(this);
}

ControlPanelWindow::~ControlPanelWindow()
{
  delete ui;
}

void ControlPanelWindow::ClearGridLayout(QLayout *layout) {
  if (!layout) {
    return;
  }
  while (auto item = layout->takeAt(0)) {
     delete item->widget();
     ClearGridLayout(item->layout());
  }
}

void ControlPanelWindow::OnShow(
    std::vector<ce30_pcviz::CtrlShortcut> shortcuts) {
  auto panel_layout = ui->ControlButtonGridLayout;
  if (!panel_layout) {
    return;
  }
  ClearGridLayout(panel_layout);
  ctrl_shortcut_map_ = shortcuts;
  for (int i = 0; i < shortcuts.size(); ++i) {
    auto button = new QPushButton(this);
    button->setText(QString::fromStdString(shortcuts[i].description));
    panel_layout->addWidget(button);
    connect(
        button, &QPushButton::pressed,
        [this, i](){ctrl_shortcut_map_[i].callback();});
  }
  this->show();
}
} // namespace ce30_pcviz
