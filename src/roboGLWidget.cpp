#include "roboGLWidget.h"

#include <QPainter>
#include <QDebug>
roboGLWidget::roboGLWidget(QWidget *parent) : QOpenGLWidget(parent) {}

void roboGLWidget::updateRunMap(QPixmap map){
    m_map=map;
    update();
}
roboGLWidget::~roboGLWidget() {}
void roboGLWidget::paintEvent(QPaintEvent *e) {


  QPainter painter;
  painter.begin(this);

  painter.drawPixmap(QPoint(0, 0), m_map);

  painter.end();
}
