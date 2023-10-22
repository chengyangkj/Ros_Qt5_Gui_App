/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:20:56
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:46:16
 * @FilePath: ////include/display/RobotMap.h
 */
#ifndef ROBO_MAP_H
#define ROBO_MAP_H
#include "basic/map/cost_map.h"
#include "virtual_display.h"
#include <Eigen/Dense>
namespace Display {
class RobotMap : public VirtualDisplay {
private:
  /* data */
public:
  enum MapType { kGridMap, kOccupyMap, kTrustMap, kCostMap };
  RobotMap(const MapType &type, const std::string &display_name,
           const int &z_value, std::string group_name = "");
  ~RobotMap() = default;
  bool UpdateData(const std::any &data) override;
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data);

private:
  OccupancyMap map_data_;
  CostMap cost_map_data_;
  MapType map_type_;

  QTransform transform_;
  QImage map_image_;
  Eigen::Vector3f sub_map_center_pose_;
  double sub_map_value_ = 1;

private:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  void ParseCostMap();
  void ParseGridMap();
  void ParseTrustMap();
  void ParseOccupyMap();
};
} // namespace Display
#endif
