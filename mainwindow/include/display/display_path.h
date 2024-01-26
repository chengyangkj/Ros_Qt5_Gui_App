/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-11 10:13:22
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-04-20 16:46:39
 * @FilePath:
 * ////include/display/display_demo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"
using namespace basic;
namespace Display {
class DisplayPath : public VirtualDisplay {
 private:
  QColor color_;
  QPolygonF path_points_;

 private:
  void drawPath(QPainter *painter);

  void computeBoundRect(const RobotPath &path);

 public:
  DisplayPath(const std::string &display_type, const int &z_value,
              std::string parent_name = "");
  ~DisplayPath();
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data) override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};
}  // namespace Display