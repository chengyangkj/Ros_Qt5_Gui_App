#ifndef ROBOTALGORITHM_H
#define ROBOTALGORITHM_H

namespace algo {

//#define M_PI 3.1415
//角度转弧度
inline double deg2rad(double x){
    return M_PI*x/180.0;
}
//弧度转角度
inline double rad2deg(double x ){
    return 180.0*x/M_PI;
}
struct RobotPose{
    double x;
    double y;
    double theta;
};

struct RobotSpeed{
    double vx;
    double vy;
    double w;
};
}


#endif // ALGORITHM_H
