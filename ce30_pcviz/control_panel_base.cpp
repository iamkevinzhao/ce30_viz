#include "control_panel_base.h"
#include <QPushButton>

namespace ce30_pcviz {
ControlPanelBase::ControlPanelBase(QWidget *parent) : QWidget(parent)
{

}

ControlPanelBase::~ControlPanelBase() {
  ClearGridLayout(button_grid_layout_);
}

void ControlPanelBase::ClearGridLayout(QLayout *layout) {
  if (!layout) {
    return;
  }
  while (auto item = layout->takeAt(0)) {
    delete item->widget();
    ClearGridLayout(item->layout());
  }
}

void ControlPanelBase::OnShow(
    std::vector<ce30_pcviz::CtrlShortcut> shortcuts) {
  if (!button_grid_layout_) {
    return;
  }
  auto panel_layout = button_grid_layout_;
  if (!panel_layout) {
    return;
  }
  disconnect();
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

void ControlPanelBase::SetButtonGridLayout(QGridLayout *layout) {
  button_grid_layout_ = layout;
}
} // namespace ce30_pcviz
