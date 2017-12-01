#include "config.h"
#include "utilities.h"

Config gConfig;

Config::Config()
  : full_image_size_(312, 20),
    segment_image_hv_ratio_(1.5f),
    ip_default_("192.168.1.80 "),
    port_default_("2368"),
    ip_key_("ip"),
    port_key_("port"),
    network_group_("network/"),
    device_id_(-1)

{
  // Image Display
  segment_image_size_ =
      QSize(
          full_image_size_.height() * segment_image_hv_ratio_,
          full_image_size_.height());
  full_image_hv_ratio_ = full_image_size_.width() / full_image_size_.height();

  // Ini Files
  ini_file_path_ =
      Utilities::DirFile(Utilities::DocumentsFolderPath(), "ce30_viz.ini");

  Default();
}

Config::~Config() {}

void Config::Load(const QSettings& settings) {
  // Network
  ip_ = settings.value(network_group_ + ip_key_, ip_default_).toString();
  port_ = settings.value(network_group_ + port_key_, port_default_).toString();
}

void Config::Save(QSettings& settings) {
  // Network
  settings.setValue(network_group_ + ip_key_, ip_);
  settings.setValue(network_group_ + port_key_, port_);
}

void Config::Default() {
  // Network
  ip_ = ip_default_;
  port_ = port_default_;
}
