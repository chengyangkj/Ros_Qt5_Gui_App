/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:20:56
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:46:16
 * @FilePath: ////include/display/DisplayCostMap.h
 */
#pragma once
#include <Eigen/Dense>
#include "occupancy_map.h"
#include "virtual_display.h"
namespace Display {
class DisplayCostMap : public VirtualDisplay {
 private:
  /* data */
 public:
  DisplayCostMap(const std::string &display_type, const int &z_value,
                 std::string parent_name = "");
  ~DisplayCostMap() = default;
  bool UpdateData(const std::any &data) override;
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data);

 private:
  OccupancyMap cost_map_data_;

  QTransform transform_;
  QImage map_image_;

 private:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  void ParseCostMap();
};
}  // namespace Display
