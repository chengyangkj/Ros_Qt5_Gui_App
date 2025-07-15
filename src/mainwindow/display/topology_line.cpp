/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2024-01-15 10:13:22
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2024-01-15 16:46:39
 * @FilePath: src/mainwindow/display/topology_line.cpp
 * @Description: 拓扑连接线显示类实现
 */
#include "display/topology_line.h"
#include <QGraphicsSceneMouseEvent>
#include <QtMath>
#include "logger/logger.h"

namespace Display {

TopologyLine::TopologyLine(QGraphicsItem* from_item, QGraphicsItem* to_item, 
                           bool bidirectional, const std::string &display_name)
    : QGraphicsLineItem(), start_item_(from_item), end_item_(to_item), 
      is_bidirectional_(bidirectional), display_name_(display_name) {
  
  // 设置可以接收鼠标和悬停事件
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setFlag(ItemIsSelectable, true);
  
  // 设置线段样式
  QPen pen(line_color_, line_width_);
  setPen(pen);
  
  // 设置Z值确保在最上层
  setZValue(10);
}

TopologyLine::~TopologyLine() {
}

QRectF TopologyLine::boundingRect() const {
 
  
  return QRectF(-5000, -5000, 10000, 10000);
}


void TopologyLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                         QWidget *widget) {
  // 动态获取起点和终点的实时位置
  if (!start_item_) {
    return; // 如果没有起点item，不绘制
  }
  
  QPointF start_pos = start_item_->scenePos();
  QPointF end_pos;
  
  if (is_preview_mode_) {
    // 预览模式：使用预设的终点位置
    end_pos = preview_end_pos_;
  } else if (end_item_) {
    // 正常模式：使用终点item的位置
    end_pos = end_item_->scenePos();
  } else {
    return; // 既不是预览模式也没有终点item，不绘制
  }
  
  // 设置绘制属性
  painter->setRenderHint(QPainter::Antialiasing, true);
  
  // 选择颜色和样式
  QColor color = line_color_;
  Qt::PenStyle pen_style = Qt::SolidLine;
  
  if (is_preview_mode_) {
    // 预览模式：使用灰色虚线
    color = Qt::gray;
    pen_style = Qt::DashLine;
  } else if (is_selected_) {
    color = selected_color_;
  } else if (is_highlighted_) {
    color = line_color_.lighter(150);
  }
  
  // 设置画笔
  QPen pen(color, line_width_, pen_style);
  painter->setPen(pen);
  
  // 绘制线段
  painter->drawLine(start_pos, end_pos);
  
  // 绘制箭头
  if (is_bidirectional_) {
    // 双向箭头
    drawArrow(painter, start_pos, end_pos);
    drawArrow(painter, end_pos, start_pos);
  } else {
    // 单向箭头
    drawArrow(painter, start_pos, end_pos);
  }
}

void TopologyLine::drawArrow(QPainter *painter, const QPointF &start, const QPointF &end) {
  // 计算箭头位置（在终点前一段距离）
  QPointF direction = end - start;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  if (length == 0) return;
  
  direction /= length;
  QPointF arrow_pos = end - direction * (arrow_size_ + 5); // 箭头距离终点5像素
  
  // 计算箭头的两个边
  QPointF arrow_head1 = calculateArrowHead(arrow_pos, end, M_PI / 6);
  QPointF arrow_head2 = calculateArrowHead(arrow_pos, end, -M_PI / 6);
  
  // 绘制箭头
  QPolygonF arrow;
  arrow << end << arrow_head1 << arrow_head2;
  painter->setBrush(painter->pen().color());
  painter->drawPolygon(arrow);
}

QPointF TopologyLine::calculateArrowHead(const QPointF &start, const QPointF &end, double angle) {
  QPointF direction = end - start;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  if (length == 0) return start;
  
  direction /= length;
  
  // 旋转方向向量
  qreal cos_angle = qCos(angle);
  qreal sin_angle = qSin(angle);
  QPointF rotated_dir(
    direction.x() * cos_angle - direction.y() * sin_angle,
    direction.x() * sin_angle + direction.y() * cos_angle
  );
  
  return end - rotated_dir * arrow_size_;
}

bool TopologyLine::contains(const QPointF &point) const {
  if (!start_item_ || !end_item_) {
    return false;
  }
  
  // 获取实时位置
  QPointF start_pos = start_item_->scenePos();
  QPointF end_pos = end_item_->scenePos();
  
  // 创建线段路径
  QPainterPath path;
  path.moveTo(start_pos);
  path.lineTo(end_pos);
  
  // 增加容错范围
  QPainterPathStroker stroker;
  stroker.setWidth(line_width_ + 5); // 增加5像素的点击范围
  QPainterPath strokePath = stroker.createStroke(path);
  
  return strokePath.contains(point);
}

QPainterPath TopologyLine::shape() const {
  if (!start_item_ || !end_item_) {
    return QPainterPath();
  }
  
  // 获取实时位置
  QPointF start_pos = start_item_->scenePos();
  QPointF end_pos = end_item_->scenePos();
  
  // 创建线段路径
  QPainterPath path;
  path.moveTo(start_pos);
  path.lineTo(end_pos);
  
  // 增加点击区域
  QPainterPathStroker stroker;
  stroker.setWidth(line_width_ + 5);
  return stroker.createStroke(path);
}

void TopologyLine::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    SetSelected(!is_selected_);
    LOG_INFO("TopologyLine selected: " << GetDisplayName());
  }
  QGraphicsLineItem::mousePressEvent(event);
}

void TopologyLine::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  SetHighlighted(true);
  QGraphicsLineItem::hoverEnterEvent(event);
}

void TopologyLine::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  SetHighlighted(false);
  QGraphicsLineItem::hoverLeaveEvent(event);
}

}  // namespace Display 