#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QWidget>

class DashBoard : public QWidget {
  Q_OBJECT
 public:
  enum Gear {
    kGear_1 = 1,
    kGear_2,
    kGear_3,
    kGear_4,
    kGear_5,
    kGear_6,
    kGear_7,
    kGear_8,
    kGear_D,
    kGear_N,
    kGear_P,
    kGear_R
  };

 public:
  explicit DashBoard(QWidget* parent = nullptr);

 public slots:
  void set_gear(const Gear gear);
  void set_rpm(const int rpm);
  void set_speed(const int speed);
  void set_temperature(const double temperature);
  void set_oil(const int oil);

 protected:
  void paintEvent(QPaintEvent* event);

 private:
  void draw_tachometer(QPainter& painter);   // 转速表
  void draw_speedometer(QPainter& painter);  // 迈速表
  void draw_gear(QPainter& painter);         // 挡位
  void draw_thermometer(QPainter& painter);  // 水箱温度计
  void draw_oil_meter(QPainter& painter);    // 油表

 private:
  Gear _gear{kGear_N};
  int _rpm;
  int _speed;
  double _temperature;
  int _oil;
  QWidget* parent;
};

#endif  // DASHBOARD_H
