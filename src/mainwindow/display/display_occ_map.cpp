
/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:21:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 02:49:39
 * @FilePath: ////src/display/robot_map.cpp
 */
#include "display/display_occ_map.h"
#include <QtConcurrent>
#include <algorithm>
#include <iostream>
#include "core/framework/framework.h"
#include "msg/msg_info.h"
#include "display/manager/display_factory.h"
namespace Display {
DisplayOccMap::DisplayOccMap(const std::string &display_type,
                             const int &z_value, std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  SetMoveEnable(true);
  SUBSCRIBE(MSG_ID_OCCUPANCY_MAP, [this](const OccupancyMap& data) {
    map_data_ = data;
    ParseOccupyMap();
    LOG_INFO("map update calling:" << map_image_.width() << " "
            << map_image_.height() << std::endl);
  });
}
bool DisplayOccMap::SetDisplayConfig(const std::string &config_name,
                                     const std::any &config_data) {
  if (config_name == "SubMapValue") {
    GetAnyData(double, config_data, sub_map_value_);
  } else if (config_name == "RobotPose") {
    // using type = Eigen::Vector3f;
    GetAnyData(Eigen::Vector3f, config_data, sub_map_center_pose_);
  } else {
    return false;
  }
  update();
  return true;
}
void DisplayOccMap::paint(QPainter *painter,
                          const QStyleOptionGraphicsItem *option,
                          QWidget *widget) {
  painter->drawImage(0, 0, map_image_);
}
void DisplayOccMap::ParseOccupyMap() {
  QtConcurrent::run([this]() {
    // Eigen::matrix 坐标系与QImage坐标系不同,这里行列反着遍历
    //QImage坐标系
    // **************x
    // *
    // *
    // *
    // y
    map_image_ = QImage(map_data_.Cols(), map_data_.Rows(), QImage::Format_ARGB32);

    // 遍历地图数据，设置每个像素的颜色
    for (int i = 0; i < map_data_.Cols(); i++) {
      for (int j = 0; j < map_data_.Rows(); j++) {
        double map_value = map_data_(j, i);
        QColor color;

        if (map_value > 0) {
          // 将 map_value 从 0-100 映射到透明度 0-255 范围
          int alpha = static_cast<int>(std::clamp(map_value * 2.55, 0.0, 255.0));
          color = QColor(0, 0, 0, alpha);  // 黑色, 透明度根据占据值动态调整
        } else if (map_value == 0 ) {
          // 自由区域和未知区域都设为白色
          color = Qt::white;
        } else if (map_value == -1) {
          color = Qt::gray;
        } else {
          color = Qt::white;  // 默认白色
        }

        // 使用 RGBA 颜色值绘制像素
        map_image_.setPixel(i, j, color.rgba());
      }
    }

    // 更新边界矩形
    SetBoundingRect(QRectF(0, 0, map_image_.width(), map_image_.height()));
    update();
    //以0 0点为中心
    double x, y;
    map_data_.xy2ScenePose(0, 0, x, y);
    if (!init_flag_) {
      CenterOnScene(mapToScene(x, y));
      init_flag_ = true;
    }
  });
}
void DisplayOccMap::EraseMapRange(const QPointF &pose, double range) {
  float x = pose.x();
  float y = pose.y();
  // 确保传入的坐标在图像范围内
  if (x < 0 || x >= map_image_.width() || y < 0 || y >= map_image_.height()) {
    return;
  }
  // 计算擦除范围的矩形区域
  int left = qMax(0, static_cast<int>(x - range));
  int top = qMax(0, static_cast<int>(y - range));
  int right = qMin(map_image_.width() - 1, static_cast<int>(x + range));
  int bottom = qMin(map_image_.height() - 1, static_cast<int>(y + range));

  // 循环遍历范围内的像素点，将其颜色设置为透明
  for (int i = left; i <= right; ++i) {
    for (int j = top; j <= bottom; ++j) {
      map_image_.setPixelColor(i, j, Qt::white);
    }
  }
  update();
}

void DisplayOccMap::DrawMapRange(const QPointF &pose, double range) {
  float x = pose.x();
  float y = pose.y();
  // 确保传入的坐标在图像范围内
  if (x < 0 || x >= map_image_.width() || y < 0 || y >= map_image_.height()) {
    return;
  }
  // 计算绘制范围的矩形区域
  int left = qMax(0, static_cast<int>(x - range));
  int top = qMax(0, static_cast<int>(y - range));
  int right = qMin(map_image_.width() - 1, static_cast<int>(x + range));
  int bottom = qMin(map_image_.height() - 1, static_cast<int>(y + range));

  // 循环遍历范围内的像素点，将其颜色设置为黑色
  for (int i = left; i <= right; ++i) {
    for (int j = top; j <= bottom; ++j) {
      map_image_.setPixelColor(i, j, Qt::black);
    }
  }
  update();
}

OccupancyMap DisplayOccMap::GetOccupancyMap() {
  OccupancyMap map = map_data_;

  for (int i = 0; i < map_image_.width(); i++) {
    for (int j = 0; j < map_image_.height(); j++) {
      QRgb pixelValue = map_image_.pixel(i, j);  // 获取指定位置的像素值
      QColor color(pixelValue);                  // 从像素值创建 QColor 对象

      // 提取Alpha通道值，范围是 0-255
      int alpha = color.alpha();

      // 如果颜色是黑色且 alpha > 0，表示占据栅格
      if (color == QColor(Qt::black) && alpha > 0) {
        // 将 alpha 映射回 0-100 的栅格值 (之前是将 0-100 映射到 0-255 的透明度)
        int map_value = static_cast<int>(alpha / 2.55);  // 反向还原栅格值
        map(j, i) = map_value;                           // 还原栅格数据
      }
      // 如果颜色是白色，表示自由区域或未知区域
      else if (color == QColor(Qt::white)) {
        // 原始数据可能是自由区域或未知区域
        if (alpha == 255) {
          map(j, i) = 0;  // 自由区域
        } else {
          map(j, i) = -1;  // 未知区域
        }
      } else {
        map(j, i) = -1;  // 未知区域，或其他情况
      }
    }
  }

  return map;
}

void DisplayOccMap::StartDrawLine(const QPointF &pose) {
  line_start_pose_ = pose;
}
void DisplayOccMap::EndDrawLine(const QPointF &pose, bool is_draw) {
  if (!is_draw_line_) {
    line_tmp_image_ = map_image_;
    is_draw_line_ = true;
  }
  map_image_ = line_tmp_image_;
  QPainter painter(&map_image_);
  painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawLine(line_start_pose_, pose);
  if (is_draw) {
    //结束绘制
    is_draw_line_ = false;
  }
  update();
}
void DisplayOccMap::DrawPoint(const QPointF &point) {
  QPainter painter(&map_image_);
  painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPoint(point);
  update();
}

QImage DisplayOccMap::GetMapImageRegion(const QRectF &region) {
  QRect rect = region.toRect();
  rect = rect.intersected(QRect(0, 0, map_image_.width(), map_image_.height()));
  if (rect.isEmpty()) {
    return QImage();
  }
  return map_image_.copy(rect);
}

void DisplayOccMap::RestoreMapImageRegion(const QRectF &region, const QImage &image) {
  QRect rect = region.toRect();
  rect = rect.intersected(QRect(0, 0, map_image_.width(), map_image_.height()));
  if (rect.isEmpty() || image.isNull()) {
    return;
  }
  QPainter painter(&map_image_);
  painter.drawImage(rect.topLeft(), image);
  update();
}

void DisplayOccMap::SetMapImage(const QImage &image) {
  map_image_ = image;
  update();
}
}  // namespace Display