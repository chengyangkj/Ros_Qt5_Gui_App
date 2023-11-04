#include "common/config/configManager.h"
#include <QFile>
namespace Config {
// #define CHECK_DEFALUT
configManager::configManager(/* args */) { Init(config_path_); }
void configManager::Init(const QString &config_path) {
  QFile file(config_path);
  config_path_ = config_path;
  QSettings setting(config_path_, QSettings::IniFormat);
  // init
  if (!file.exists()) {
    setting.setValue("NavGoal/Topic", "/goal_pose");
    setting.setValue("Reloc/Topic", "/initialpose");
    setting.setValue("Map/Topic", "/map");
    setting.setValue("LocalCostMap/Topic", "/local_costmap/costmap");
    setting.setValue("GlobalCostMap/Topic", "/global_costmap/costmap");
    setting.setValue("LaserScan/Topic", "/scan");
    setting.setValue("GlobalPlan/Topic", "/plan");
    setting.setValue("LocalPlan/Topic", "/local_plan");
    setting.setValue("Odometry/Topic", "/odom");
    setting.setValue("Speed/Topic", "/cmd_vel");
  }
}
configManager::~configManager() {}
std::string configManager::GetTopicName(const std::string &frame_name) {
  QSettings setting(config_path_, QSettings::IniFormat);
  return setting.value(QString::fromStdString(frame_name) + "/Topic", "")
      .toString()
      .toStdString();
}
} // namespace Config