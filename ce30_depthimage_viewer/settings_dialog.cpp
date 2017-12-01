#include "settings_dialog.h"
#include "ui_settings_dialog.h"
#include "utilities.h"
#include "config.h"

SettingsDialog::SettingsDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::SettingsDialog)
{
  ui->setupUi(this);

  ip_line_edits_.push_back(ui->ip1LineEdit);
  ip_line_edits_.push_back(ui->ip2LineEdit);
  ip_line_edits_.push_back(ui->ip3LineEdit);
  ip_line_edits_.push_back(ui->ip4LineEdit);

  for (auto& i : ip_line_edits_) {
    i->setValidator(new QIntValidator(0, 255, this));
  }
  ui->portLineEdit->setValidator(new QIntValidator(0, 9999, this));
  ui->deviceIDLineEdit->setValidator(new QIntValidator(0, 10000, this));

  LoadConfigUI(gConfig);
}

SettingsDialog::~SettingsDialog()
{
  delete ui;
}

void SettingsDialog::LoadConfigUI(const Config& config) {
  auto ips = Utilities::IPSegments(config.GetIP());
  Q_ASSERT(ips.size() == ip_line_edits_.size());
  for (int i = 0; i < ip_line_edits_.size(); ++i) {
    ip_line_edits_[i]->setText(ips[i]);
  }
  ui->portLineEdit->setText(config.GetPort());
  if (!gConfig.HoldValidDeviceID()) {
    ui->deviceIDApplyCheckBox->setChecked(false);
    ui->deviceIDLineEdit->clear();
  } else {
    ui->deviceIDApplyCheckBox->setChecked(true);
    ui->deviceIDLineEdit->setText(QString::number(gConfig.GetDeviceID()));
  }
}

void SettingsDialog::SaveConfigUI(Config& config) {
  config.SetIP(
        ui->ip1LineEdit->text() + "." + ui->ip2LineEdit->text() + "." +
        ui->ip3LineEdit->text() + "." + ui->ip4LineEdit->text());
  config.SetPort(ui->portLineEdit->text());

  if (ui->deviceIDApplyCheckBox->isChecked()) {
    if (ui->deviceIDLineEdit->text().isEmpty()) {
      ui->deviceIDApplyCheckBox->setChecked(false);
    } else {
      config.SetDeviceID(ui->deviceIDLineEdit->text().toInt());
    }
  } else {
    ui->deviceIDLineEdit->clear();
  }
}

void SettingsDialog::on_okPushButton_clicked()
{
  SaveConfigUI(gConfig);
  close();
}

void SettingsDialog::on_defaultPushButton_clicked()
{
  Config config;
  LoadConfigUI(config);
}

void SettingsDialog::on_cancelPushButton_clicked()
{
  close();
}
