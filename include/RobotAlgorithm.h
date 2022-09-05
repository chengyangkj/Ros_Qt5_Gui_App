#ifndef ROBOTALGORITHM_H
#define ROBOTALGORITHM_H
#include <math.h>

#include <QColor>
struct RobotPose {
  double x{0};
  double y{0};
  double theta{0};
};
struct RobotSpeed {
  double vx{0};
  double vy{0};
  double w{0};
};
struct RobotState:public RobotPose,RobotSpeed
{
  
};

//角度转弧度
inline double deg2rad(double x) { return M_PI * x / 180.0; }
//弧度转角度
inline double rad2deg(double x) { return 180.0 * x / M_PI; }
#endif  // ROBOTALGORITHM_H
