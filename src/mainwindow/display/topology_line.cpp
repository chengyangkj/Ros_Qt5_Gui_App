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
#include "display/virtual_display.h"

namespace Display {

TopologyLine::TopologyLine(QGraphicsItem* from_item, QGraphicsItem* to_item, 
                           const std::string &display_name)
    : VirtualDisplay(DISPLAY_TOPOLINE, 8, "MAP", display_name), start_item_(from_item), end_item_(to_item), 
      is_part_of_bidirectional_(false) {
  
  // 设置可以接收鼠标和悬停事件
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setFlag(ItemIsSelectable, true);
  
  // 设置初始bounding rect，会动态更新
  updateBoundingRect();
  SetScaleEnable(false);
  setZValue(8);
  
  // 监听关联item的位置变化
  connectToItems();
}

TopologyLine::~TopologyLine() {
}

void TopologyLine::connectToItems() {
  // 监听起点item的位置变化
  if (start_item_) {
    VirtualDisplay* start_display = dynamic_cast<VirtualDisplay*>(start_item_);
    if (start_display) {
      connect(start_display, &VirtualDisplay::signalPositionChanged, this, [this]() {
        updateBoundingRect();
      });
    }
  }
  
  // 监听终点item的位置变化
  if (end_item_) {
    VirtualDisplay* end_display = dynamic_cast<VirtualDisplay*>(end_item_);
    if (end_display) {
      connect(end_display, &VirtualDisplay::signalPositionChanged, this, [this]() {
        updateBoundingRect();
      });
    }
  }
}




void TopologyLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                         QWidget *widget) {
  // 动态获取起点和终点的实时位置
  if (!start_item_) {
    return; // 如果没有起点item，不绘制
  }
  
  // 动态更新bounding rect
  updateBoundingRect();
  
  // 确保边界矩形有效，如果无效则重新计算
  if (bounding_rect_.isEmpty() || bounding_rect_.width() < 10 || bounding_rect_.height() < 10) {
    updateBoundingRect();
  }
  
  QPointF start_pos = mapFromScene(start_item_->scenePos());
  QPointF end_pos;
  
  if (is_preview_mode_) {
    // 预览模式：使用预设的终点位置
    end_pos = mapFromScene(preview_end_pos_);
  } else if (end_item_) {
    // 正常模式：使用终点item的位置
    end_pos = mapFromScene(end_item_->scenePos());
  } else {
    return; // 既不是预览模式也没有终点item，不绘制
  }
  
  // 如果是双向连接的一部分，添加并行偏移
  if (is_part_of_bidirectional_ && !is_preview_mode_) {
    // 先转换回场景坐标计算偏移
    QPointF scene_start = start_item_->scenePos();
    QPointF scene_end = end_item_->scenePos();
    auto offset_positions = calculateOffsetPositions(scene_start, scene_end);
    // 再转换为本地坐标
    start_pos = mapFromScene(offset_positions.first);
    end_pos = mapFromScene(offset_positions.second);
  }
  
  // 设置绘制属性
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
  
  // 选择颜色和样式
  QColor main_color = line_color_;
  Qt::PenStyle pen_style = Qt::SolidLine;
  bool draw_shadow = true;
  
  if (is_preview_mode_) {
    // 预览模式：使用灰色动态虚线，无阴影
    main_color = preview_color_;
    pen_style = Qt::CustomDashLine;
    draw_shadow = false;
  } else if (is_selected_) {
    main_color = selected_color_;
    // 选中状态添加脉动效果
    main_color = QColor(main_color.red(), main_color.green(), main_color.blue(), 
                       180 + 75 * qSin(animation_offset_));
  } else if (is_highlighted_) {
    main_color = highlighted_color_;
  }
  
  // 绘制阴影效果
  if (draw_shadow && !is_preview_mode_) {
    QPen shadow_pen(shadow_color_, line_width_ + 2, pen_style);
    shadow_pen.setCapStyle(Qt::RoundCap);
    shadow_pen.setJoinStyle(Qt::RoundJoin);
    painter->setPen(shadow_pen);
    painter->drawLine(start_pos + QPointF(2, 2), end_pos + QPointF(2, 2));
  }
  
  // 创建渐变效果
  QLinearGradient gradient(start_pos, end_pos);
  if (is_preview_mode_) {
    gradient.setColorAt(0, main_color);
    gradient.setColorAt(1, main_color.darker(120));
  } else {
    gradient.setColorAt(0, main_color);
    gradient.setColorAt(0.5, main_color.lighter(120));
    gradient.setColorAt(1, main_color.darker(110));
  }
  
  // 设置主画笔
  QPen main_pen(QBrush(gradient), line_width_, pen_style);
  main_pen.setCapStyle(Qt::RoundCap);
  main_pen.setJoinStyle(Qt::RoundJoin);
  
  // 为预览模式设置动态虚线模式
  if (is_preview_mode_) {
    QVector<qreal> dash_pattern;
    qreal offset = fmod(animation_offset_, 20.0); // 20像素循环
    dash_pattern << 8 << 6; // 8像素实线，6像素空白
    main_pen.setDashPattern(dash_pattern);
    main_pen.setDashOffset(offset);
  }
  
  painter->setPen(main_pen);
  
  // 绘制主线段（总是单条线段）
  painter->drawLine(start_pos, end_pos);
  
  // 选中状态添加流光效果
  if (is_selected_ && !is_preview_mode_) {
    drawFlowingLight(painter, start_pos, end_pos, main_color);
  }
  
  // 绘制动态箭头
  if (is_preview_mode_) {
    // 预览模式：固定箭头在终点
    drawStaticArrow(painter, start_pos, end_pos, main_color);
  } else {
    // 正常模式：动态滚动箭头
    drawMovingArrows(painter, start_pos, end_pos, main_color);
  }
}

void TopologyLine::drawStaticArrow(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color) {
  // 计算箭头位置（在终点前一段距离）
  QPointF direction = end - start;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  if (length == 0) return;
  
  direction /= length;
  QPointF arrow_tip = end - direction * 8; // 箭头尖端距离终点8像素
  
  // 计算箭头的各个点（更宽更美观的箭头）
  QPointF arrow_base1 = calculateArrowHead(arrow_tip, end, M_PI / 4);    // 45度角
  QPointF arrow_base2 = calculateArrowHead(arrow_tip, end, -M_PI / 4);   // -45度角
  QPointF arrow_mid = arrow_tip - direction * (arrow_size_ * 0.3); // 箭头中部凹陷点
  
  // 创建箭头多边形（更美观的形状）
  QPolygonF arrow;
  arrow << end << arrow_base1 << arrow_mid << arrow_base2;
  
  // 创建箭头渐变
  QLinearGradient arrow_gradient(arrow_mid, end);
  arrow_gradient.setColorAt(0, color.darker(120));
  arrow_gradient.setColorAt(0.5, color);
  arrow_gradient.setColorAt(1, color.lighter(130));
  
  // 保存当前状态
  painter->save();
  
  // 绘制箭头阴影（如果不是预览模式）
  if (!is_preview_mode_) {
    QPolygonF shadow_arrow = arrow.translated(1.5, 1.5);
    painter->setBrush(shadow_color_);
    painter->setPen(Qt::NoPen);
    painter->drawPolygon(shadow_arrow);
  }
  
  // 绘制主箭头
  painter->setBrush(QBrush(arrow_gradient));
  painter->setPen(QPen(color.darker(150), 1, Qt::SolidLine));
  painter->drawPolygon(arrow);
  
  // 恢复状态
  painter->restore();
}

void TopologyLine::drawMovingArrows(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color) {
  QPointF direction = end - start;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  if (length < arrow_size_) return;
  
  direction /= length;
  
  // 计算箭头间距和数量
  qreal arrow_spacing = 60.0; // 箭头间距
  qreal arrow_size = arrow_size_ * 0.8; // 稍小的箭头
  int num_arrows = static_cast<int>(length / arrow_spacing) + 1;
  
  // 计算动画偏移
  qreal anim_offset = fmod(animation_offset_ * 6, arrow_spacing); // 控制移动速度（减慢箭头滚动）
  
  painter->save();
  
  // 绘制箭头
  for (int i = 0; i < num_arrows; ++i) {
    qreal t = (i * arrow_spacing + anim_offset) / length;
    if (t > 1.0) t = fmod(t, 1.0); // 循环
    
    QPointF arrow_pos = start + direction * (t * length);
    
    // 计算箭头透明度（渐变效果）
    qreal alpha = 0.3 + 0.7 * qSin(t * M_PI * 2 + animation_offset_);
    alpha = qMax(0.2, qMin(1.0, alpha));
    
    QColor arrow_color = color;
    arrow_color.setAlphaF(alpha);
    
    drawSingleMovingArrow(painter, arrow_pos, direction, arrow_color, arrow_size);
  }
  
  painter->restore();
}

void TopologyLine::drawSingleMovingArrow(QPainter *painter, const QPointF &pos, const QPointF &direction, const QColor &color, qreal size) {
  // 计算箭头的各个点
  QPointF tip = pos + direction * size * 0.5;
  QPointF base1 = pos + QPointF(-direction.y(), direction.x()) * size * 0.3;
  QPointF base2 = pos + QPointF(direction.y(), -direction.x()) * size * 0.3;
  QPointF back = pos - direction * size * 0.3;
  
  // 创建箭头形状
  QPolygonF arrow;
  arrow << tip << base1 << back << base2;
  
  // 绘制箭头
  painter->setBrush(color);
  painter->setPen(QPen(color.darker(150), 1));
  painter->drawPolygon(arrow);
}

void TopologyLine::drawFlowingLight(QPainter *painter, const QPointF &start, const QPointF &end, const QColor &color) {
  QPointF direction = end - start;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  if (length == 0) return;
  
  direction /= length;
  
  painter->save();
  
  // 创建多个流光点
  int num_lights = 3;
  qreal light_spacing = length / (num_lights + 1);
  
  for (int i = 0; i < num_lights; ++i) {
    // 计算流光位置（循环移动）
    qreal offset = fmod(animation_offset_ * 12 + i * light_spacing, length);
    QPointF light_pos = start + direction * offset;
    
    // 计算透明度（脉动效果）
    qreal alpha = 0.3 + 0.7 * qSin(animation_offset_ * 2 + i * M_PI / 2);
    alpha = qMax(0.1, qMin(0.8, alpha));
    
    // 创建径向渐变
    QRadialGradient gradient(light_pos, 8);
    QColor light_color = color.lighter(180);
    light_color.setAlphaF(alpha);
    gradient.setColorAt(0, light_color);
    light_color.setAlphaF(0);
    gradient.setColorAt(1, light_color);
    
    // 绘制流光
    painter->setBrush(QBrush(gradient));
    painter->setPen(Qt::NoPen);
    painter->drawEllipse(light_pos, 6, 6);
  }
  
  painter->restore();
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
  
  // 转换为本地坐标
  QPointF start_pos = mapFromScene(start_item_->scenePos());
  QPointF end_pos = mapFromScene(end_item_->scenePos());
  
  // 如果是双向连接的一部分，应用相同的偏移
  if (is_part_of_bidirectional_) {
    QPointF scene_start = start_item_->scenePos();
    QPointF scene_end = end_item_->scenePos();
    auto offset_positions = calculateOffsetPositions(scene_start, scene_end);
    start_pos = mapFromScene(offset_positions.first);
    end_pos = mapFromScene(offset_positions.second);
  }
  
  QPainterPath path;
  
  // 创建单条线段路径
  path.moveTo(start_pos);
  path.lineTo(end_pos);
  
  // 增加容错范围
  QPainterPathStroker stroker;
  stroker.setWidth(line_width_ + 8); // 增加点击范围
  QPainterPath strokePath = stroker.createStroke(path);
  
  return strokePath.contains(point);
}

QPainterPath TopologyLine::shape() const {
  if (!start_item_ || !end_item_) {
    return QPainterPath();
  }
  
  // 转换为本地坐标
  QPointF start_pos = mapFromScene(start_item_->scenePos());
  QPointF end_pos = mapFromScene(end_item_->scenePos());
  
  // 如果是双向连接的一部分，应用相同的偏移
  if (is_part_of_bidirectional_) {
    QPointF scene_start = start_item_->scenePos();
    QPointF scene_end = end_item_->scenePos();
    auto offset_positions = calculateOffsetPositions(scene_start, scene_end);
    start_pos = mapFromScene(offset_positions.first);
    end_pos = mapFromScene(offset_positions.second);
  }
  
  QPainterPath path;
  
  // 创建单条线段路径
  path.moveTo(start_pos);
  path.lineTo(end_pos);
  
  // 增加点击区域
  QPainterPathStroker stroker;
  stroker.setWidth(line_width_ + 8);
  return stroker.createStroke(path);
}

void TopologyLine::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  SetHighlighted(false);
  QGraphicsItem::mousePressEvent(event);
}

void TopologyLine::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  SetHighlighted(true);
  QGraphicsItem::hoverEnterEvent(event);
}

void TopologyLine::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  SetHighlighted(false);
  QGraphicsItem::hoverLeaveEvent(event);
}

std::pair<QPointF, QPointF> TopologyLine::calculateOffsetPositions(const QPointF &start_pos, const QPointF &end_pos) const {
  QPointF direction = end_pos - start_pos;
  qreal length = qSqrt(direction.x() * direction.x() + direction.y() * direction.y());
  
  if (length == 0) {
    return std::make_pair(start_pos, end_pos);
  }
  
  direction /= length;
  
  // 计算垂直于线段的偏移向量
  QPointF perpendicular(-direction.y(), direction.x());
  qreal offset_distance = 4.0; // 增加偏移距离，确保能看到分离效果
  
  // 获取起点和终点的显示对象
  VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(start_item_);
  VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(end_item_);
  
  if (!from_display || !to_display) {
    return std::make_pair(start_pos, end_pos);
  }
  
  std::string from_name = from_display->GetDisplayName();
  std::string to_name = to_display->GetDisplayName();
  
  // 对于任意两个点A和B，我们统一以字典序较小的点为基准来计算偏移
  // 这样A->B和B->A会使用相同的基础偏移方向，然后通过正负号来区分
  
  std::string min_name = std::min(from_name, to_name);
  std::string max_name = std::max(from_name, to_name);
  
  // 计算标准化方向（总是从字典序小的点指向字典序大的点）
  QPointF normalized_start, normalized_end;
  if (from_name == min_name) {
    // 当前线段方向与标准方向一致
    normalized_start = start_pos;
    normalized_end = end_pos;
  } else {
    // 当前线段方向与标准方向相反
    normalized_start = end_pos;
    normalized_end = start_pos;
  }
  
  QPointF normalized_direction = normalized_end - normalized_start;
  qreal normalized_length = qSqrt(normalized_direction.x() * normalized_direction.x() + normalized_direction.y() * normalized_direction.y());
  if (normalized_length > 0) {
    normalized_direction /= normalized_length;
  }
  
  // 计算标准化的垂直向量
  QPointF normalized_perpendicular(-normalized_direction.y(), normalized_direction.x());
  
  // 根据当前线段的方向决定偏移方向
  QPointF offset;
  if (from_name == min_name) {
    // 从小到大的线段，向右偏移
    offset = normalized_perpendicular * offset_distance;
  } else {
    // 从大到小的线段，向左偏移
    offset = normalized_perpendicular * (-offset_distance);
  }
  
  
  return std::make_pair(start_pos + offset, end_pos + offset);
}

void TopologyLine::advance(int step) {
  // 更新动画偏移量
  animation_offset_ += 0.08; // 控制动画速度（减慢速度）
  if (animation_offset_ > 2 * M_PI) {
    animation_offset_ = 0.0;
  }
  
  // 更频繁地更新边界矩形，确保移动时线段不会消失
  static int frame_count = 0;
  frame_count++;
  if (frame_count % 2 == 0) { // 每2帧更新一次，提高更新频率
    updateBoundingRect();
  }
  
  // 正常模式始终有动画，预览模式和选中状态有特殊效果
  update();
}

QRectF TopologyLine::calculateDynamicBoundingRect() const {
  if (!start_item_) {
    return QRectF(0, 0, 1, 1);
  }
  
  QPointF start_pos = start_item_->scenePos();
  QPointF end_pos;
  
  if (is_preview_mode_) {
    end_pos = preview_end_pos_;
  } else if (end_item_) {
    end_pos = end_item_->scenePos();
  } else {
    return QRectF(start_pos.x()-50, start_pos.y()-50, 100, 100);
  }
  
  // 计算偏移后的位置
  if (is_part_of_bidirectional_ && !is_preview_mode_) {
    auto offset_positions = calculateOffsetPositions(start_pos, end_pos);
    start_pos = offset_positions.first;
    end_pos = offset_positions.second;
  }
  
  // 计算包含起点和终点的矩形，并添加足够的边距
  qreal margin = 100; // 增加更大的边距确保完整显示，包括箭头和阴影
  qreal left = qMin(start_pos.x(), end_pos.x()) - margin;
  qreal top = qMin(start_pos.y(), end_pos.y()) - margin;
  qreal right = qMax(start_pos.x(), end_pos.x()) + margin;
  qreal bottom = qMax(start_pos.y(), end_pos.y()) + margin;
  
  // 确保边界矩形有最小尺寸，防止线段消失
  qreal min_width = 200;
  qreal min_height = 200;
  qreal width = right - left;
  qreal height = bottom - top;
  
  if (width < min_width) {
    qreal center_x = (left + right) / 2;
    left = center_x - min_width / 2;
    right = center_x + min_width / 2;
  }
  
  if (height < min_height) {
    qreal center_y = (top + bottom) / 2;
    top = center_y - min_height / 2;
    bottom = center_y + min_height / 2;
  }
  
  // 转换为相对于自身位置的本地坐标系
  QPointF current_pos = pos();
  return QRectF(left - current_pos.x(), top - current_pos.y(), 
                right - left, bottom - top);
}

void TopologyLine::updateBoundingRect() {
  prepareGeometryChange(); // 告诉场景几何形状即将改变
  QRectF new_rect = calculateDynamicBoundingRect();
  
  // 确保边界矩形有效，即使为空也要设置一个最小矩形
  if (new_rect.isEmpty()) {
    // 如果计算出的矩形为空，使用一个默认的最小矩形
    QPointF current_pos = pos();
    new_rect = QRectF(-100, -100, 200, 200);
  }
  
  SetBoundingRect(new_rect);
  // 强制更新显示
  update();
}

bool TopologyLine::UpdateData(const std::any &data) {
  // TopologyLine不需要从外部数据更新，只需要跟随关联的item位置
  updateBoundingRect();
  return true;
}

}  // namespace Display 