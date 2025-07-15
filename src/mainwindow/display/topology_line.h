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
#include <QGraphicsLineItem>
#include "virtual_display.h"
#include "map/topology_map.h"

using namespace basic;
namespace Display {
class TopologyLine : public QGraphicsLineItem {
  // 移除Q_OBJECT宏，因为QGraphicsLineItem不继承自QObject
  
 private:
  QColor line_color_{Qt::blue};
  QColor selected_color_{Qt::red};
  bool is_bidirectional_{false};
  bool is_selected_{false};
  int line_width_{3};
  int arrow_size_{15};
  bool is_highlighted_{false};
  std::string display_name_;
  
  // 关联的点位显示对象
  QGraphicsItem* start_item_{nullptr};
  QGraphicsItem* end_item_{nullptr};
  
  // 预览模式相关
  bool is_preview_mode_{false};
  QPointF preview_end_pos_;

 private:
  void drawArrow(QPainter *painter, const QPointF &start, const QPointF &end);
  QPointF calculateArrowHead(const QPointF &start, const QPointF &end, double angle);
  QRectF boundingRect() const override;
 public:
  TopologyLine(QGraphicsItem* from_item, QGraphicsItem* to_item = nullptr, 
               bool bidirectional = false, const std::string &display_name = "");
  ~TopologyLine();
  
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
  bool IsBidirectional() const { return is_bidirectional_; }
  std::string GetDisplayName() const { return display_name_; }
  
  // 预览模式控制
  void SetPreviewMode(bool preview_mode) { is_preview_mode_ = preview_mode; update(); }
  bool IsPreviewMode() const { return is_preview_mode_; }
  void SetPreviewEndPos(const QPointF &pos) { preview_end_pos_ = pos; update(); }
  
  // 重写点击检测
  bool contains(const QPointF &point) const override;
  QPainterPath shape() const override;

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
};

}  // namespace Display 