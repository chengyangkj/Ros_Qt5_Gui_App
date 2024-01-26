#pragma once
#include <QApplication>
#include <QGraphicsView>
#include <QMouseEvent>

class ViewManager : public QGraphicsView {
 public:
  ViewManager(QWidget *parent = nullptr);

 protected:
  void mouseMoveEvent(QMouseEvent *event) override;

  void enterEvent(QEvent *event) override;

  void leaveEvent(QEvent *event) override;
};
