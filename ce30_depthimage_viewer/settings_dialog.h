#ifndef SETTINGS_DIALOG_H
#define SETTINGS_DIALOG_H

#include <QDialog>
#include <QVector>
#include <QLineEdit>
#include "config.h"

namespace Ui {
  class SettingsDialog;
}

class SettingsDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SettingsDialog(QWidget *parent = 0);
  ~SettingsDialog();

private slots:
  void on_okPushButton_clicked();

  void on_defaultPushButton_clicked();

  void on_cancelPushButton_clicked();

private:
  void LoadConfigUI(const Config& config);
  void SaveConfigUI(Config& config);
  Ui::SettingsDialog *ui;
  QVector<QLineEdit*> ip_line_edits_;
};

#endif // SETTINGS_DIALOG_H
