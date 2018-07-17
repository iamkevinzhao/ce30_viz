#include "control_panel_widget.h"
#include "ui_control_panel_widget.h"

namespace ce30_pcviz {
ControlPanelWidget::ControlPanelWidget(QWidget *parent) :
  ControlPanelBase(parent),
  ui(new Ui::ControlPanelWidget)
{
  ui->setupUi(this);
  SetButtonGridLayout(ui->ControlButtonGridLayout);
}

ControlPanelWidget::~ControlPanelWidget()
{
  delete ui;
}

} // namespace ce30_pcviz
