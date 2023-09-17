/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:06:32
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-27 16:54:54
 * @FilePath: /ROS2_Qt5_Gui_App/src/roboGLWidget.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "roboGLWidget.h"

#include <QDebug>
#include <QPainter>
roboGLWidget::roboGLWidget(QWidget *parent) : QOpenGLWidget(parent) {}

void roboGLWidget::updateRunMap(QPixmap map) {
  m_map = map;
  update();
}
roboGLWidget::~roboGLWidget() {}
void roboGLWidget::paintEvent(QPaintEvent *e) {
  QPainter painter;
  painter.begin(this);
  painter.setBrush(Qt::white);

  painter.drawRect(this->rect());
  // m_map.load("/home/fr1511b/图片/dingtalkgov_qt_clipbord_pic_3.png");
  painter.drawPixmap(QPoint(0, 0), m_map);

  painter.end();
}
