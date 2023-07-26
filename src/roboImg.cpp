#include "roboImg.h"

#include "QDebug"
roboImg::roboImg() {
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
  moveBy(0, 0);
  m_robotImg.load("://images/dir_16.png");
  QMatrix matrix;
  matrix.rotate(45);
  m_robotImg = m_robotImg.transformed(matrix, Qt::SmoothTransformation);
}
QRectF roboImg::boundingRect() const {
  return QRectF(0, 0, m_robotImg.width(), m_robotImg.height());
}
void roboImg::setRobotColor(eRobotColor color) {
  switch (color) {
    case eRobotColor::blue: {
      m_robotImg.load("://images/robot_blue.png");
    } break;
    case eRobotColor::red: {
      m_robotImg.load("://images/robot_red.png");
    } break;
    case eRobotColor::yellow: {
      m_robotImg.load("://images/robot_yellow.png");
    } break;
  }
  QMatrix matrix;
  matrix.rotate(90);
  m_robotImg = m_robotImg.transformed(matrix, Qt::SmoothTransformation);
}
void roboImg::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
  painter->setRenderHint(QPainter::Antialiasing, true);  // 设置反锯齿 反走样
  painter->save();
  painter->rotate(rad2deg(-m_currRobotPose.theta));
  painter->drawPixmap(-m_robotImg.width() / 2, -m_robotImg.height() / 2,
                      m_robotImg);
  painter->restore();
}
void roboImg::updatePose(RobotPose pose) {
  m_currRobotPose = pose;
  update();
}
