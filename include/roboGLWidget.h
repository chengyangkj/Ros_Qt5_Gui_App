#ifndef ROBOT_GLWIDGET
#define ROBOT_GLWIDGET
#include <QOpenGLWidget>
class roboGLWidget :public QOpenGLWidget  {
  Q_OBJECT
 private:
  void paintEvent(QPaintEvent *e);
public slots:
  void updateRunMap(QPixmap map);
private:
  QPixmap m_map;
 public:
  roboGLWidget(QWidget *parent=nullptr);
  ~roboGLWidget();
};


#endif  // !ROBOT_GLWIDGET
