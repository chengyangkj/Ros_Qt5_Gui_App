#include "roboGLWidget.h"

#include <QPainter>
roboGLWidget::roboGLWidget(QWidget *parent) : QOpenGLWidget(parent) {}

roboGLWidget::~roboGLWidget() {}
void roboGLWidget::paintEvent(QPaintEvent *e) {
  QPainter painter;
  painter.begin(this);

//  QImage img;
//  img.load("/home/chengyangkj/hourse.pgm");
  // painter.drawImage(QPoint(0, 0), img);

  painter.end();
}
