/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:20:56
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:46:16
 * @FilePath: ////include/display/DisplayOccMap.h
 */
#ifndef ROBO_MAP_H
#define ROBO_MAP_H
#include "occupancy_map.h"
#include "virtual_display.h"
#include <Eigen/Dense>
namespace Display {
class DisplayOccMap : public VirtualDisplay {
private:
  /* data */
public:
  DisplayOccMap(const std::string &display_name, const int &z_value,
                std::string parent_name = "");
  ~DisplayOccMap() = default;
  bool UpdateData(const std::any &data) override;
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data);

private:
  OccupancyMap map_data_;

  QTransform transform_;
  QImage map_image_;
  Eigen::Vector3f sub_map_center_pose_;
  double sub_map_value_ = 1;

private:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  void ParseOccupyMap();
};
} // namespace Display
#endif
