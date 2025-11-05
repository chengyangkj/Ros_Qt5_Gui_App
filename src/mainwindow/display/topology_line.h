/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2024-01-15 10:13:22
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2024-01-15 16:46:39
 * @FilePath: src/mainwindow/display/topology_line.h
 * @Description: 拓扑连接线显示类
 */
#pragma once
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QPainter>
#include <QPainterPath>
#include "virtual_display.h"
#include "map/topology_map.h"

using namespace basic;
namespace Display {
class TopologyLine : public VirtualDisplay {
  
 private:
  // 现代化配色方案
  QColor line_color_{QColor(64, 158, 255)};        // 现代蓝色
  QColor selected_color_{QColor(255, 107, 129)};   // 现代红色
  QColor highlighted_color_{QColor(135, 206, 250)}; // 高亮蓝色
  QColor preview_color_{QColor(160, 160, 160)};     // 预览灰色
  QColor shadow_color_{QColor(0, 0, 0, 60)};        // 阴影颜色
  
  bool is_part_of_bidirectional_{false};              // 是否是双向连接的一部分
  bool is_selected_{false};
  int line_width_{4};                               // 稍微加粗
  int arrow_size_{20};                              // 加大箭头
  bool is_highlighted_{false};
  
  // 关联的点位显示对象
  QGraphicsItem* start_item_{nullptr};
  QGraphicsItem* end_item_{nullptr};
  
  // 预览模式相关
  bool is_preview_mode_{false};
  QPointF preview_end_pos_;
  
  // 动画效果相关
  qreal animation_offset_{0.0};

 private:
  void drawStaticArrow(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color);
  void drawMovingArrows(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color);
  void drawSingleMovingArrow(QPainter *painter, const QPointF &pos, const QPointF &direction, const QColor &color, qreal size);
  void drawFlowingLight(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color);
  QPointF calculateArrowHead(const QPointF &start, const QPointF &end, double angle);
  QRectF calculateDynamicBoundingRect() const;
  void connectToItems();
  
  // 计算双向连接的偏移位置
  std::pair<QPointF, QPointF> calculateOffsetPositions(const QPointF &start_pos, const QPointF &end_pos) const;
 public:
  TopologyLine(QGraphicsItem* from_item, QGraphicsItem* to_item = nullptr, 
               const std::string &display_name = "");
  ~TopologyLine();
  
  void updateBoundingRect();
  
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  
  // 选中状态
  void SetSelected(bool selected) { is_selected_ = selected; update(); }
  bool IsSelected() const { return is_selected_; }
  
  // 高亮状态
  void SetHighlighted(bool highlighted) { is_highlighted_ = highlighted; update(); }
  bool IsHighlighted() const { return is_highlighted_; }
  
  // 获取连接信息
  QGraphicsItem* GetFromItem() const { return start_item_; }
  QGraphicsItem* GetToItem() const { return end_item_; }
  bool IsPartOfBidirectional() const { return is_part_of_bidirectional_; }
  
  // 设置是否为双向连接的一部分
  void SetPartOfBidirectional(bool is_part_of_bidirectional) { 
    is_part_of_bidirectional_ = is_part_of_bidirectional; 
    update(); 
  }
  
  // 预览模式控制
  void SetPreviewMode(bool preview_mode) { is_preview_mode_ = preview_mode; update(); }
  bool IsPreviewMode() const { return is_preview_mode_; }
  void SetPreviewEndPos(const QPointF &pos) { preview_end_pos_ = pos; update(); }
  
  // 重写点击检测
  bool contains(const QPointF &point) const override;
  QPainterPath shape() const override;
  
  // 动画支持
  void advance(int step) override;

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
};

}  // namespace Display 