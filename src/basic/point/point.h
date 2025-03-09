/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-26 10:06:31
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-26 14:51:40
 * @FilePath: /ros_qt5_gui_app/include/ point.h
 * @Description: this Code reference from: openslam_gmapping,
 * "openslam_gmapping", commit d3f4c1b,
 * https://github.com/OpenSLAM-org/openslam_gmapping
 */

#ifndef _POINT_H_
#define _POINT_H_
#include <assert.h>
#include <math.h>

#include <iostream>

#define DEBUG_STREAM cerr << __PRETTY_FUNCTION__ << ":"  // FIXME

namespace basic {

/*结构体点*/
template <class T>
struct point {
  inline point() : x(0), y(0) {}
  inline point(T _x, T _y) : x(_x), y(_y) {}
  T x, y;
};

/*点的 + 运算符的重载*/
template <class T>
inline point<T> operator+(const point<T> &p1, const point<T> &p2) {
  return point<T>(p1.x + p2.x, p1.y + p2.y);
}

/*点的 - 运算符的重载*/
template <class T>
inline point<T> operator-(const point<T> &p1, const point<T> &p2) {
  return point<T>(p1.x - p2.x, p1.y - p2.y);
}

/*点的 * 运算符的重载  乘以一个数*/
template <class T>
inline point<T> operator*(const point<T> &p, const T &v) {
  return point<T>(p.x * v, p.y * v);
}

/*点的 * 运算符的重载  乘以一个数*/
template <class T>
inline point<T> operator*(const T &v, const point<T> &p) {
  return point<T>(p.x * v, p.y * v);
}

/*点的 * 运算符的重载  point的内积*/
template <class T>
inline T operator*(const point<T> &p1, const point<T> &p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

/*带方向的点  即除了位置之外，还有角度，可以理解为机器人的位姿*/
template <class T, class A>
struct orientedpoint : public point<T> {
  inline orientedpoint() : point<T>(0, 0), theta(0){};
  inline orientedpoint(const point<T> &p);
  inline orientedpoint(T x, T y, A _theta) : point<T>(x, y), theta(_theta) {}
  inline void normalize();

  inline orientedpoint<T, A> rotate(A alpha) {
    T s = sin(alpha), c = cos(alpha);
    A a = alpha + theta;
    a = atan2(sin(a), cos(a));
    return orientedpoint(c * this->x - s * this->y, s * this->x + c * this->y,
                         a);
  }
  inline double mod() { return sqrt(this->x * this->x + this->y * this->y); }
  A theta;
};

/*角度归一化 即把角度化成-PI~PI*/
template <class T, class A>
void orientedpoint<T, A>::normalize() {
  if (theta >= -M_PI && theta < M_PI)
    return;

  int multiplier = (int)(theta / (2 * M_PI));
  theta = theta - multiplier * 2 * M_PI;
  if (theta >= M_PI)
    theta -= 2 * M_PI;
  if (theta < -M_PI)
    theta += 2 * M_PI;
}

/*构造函数*/
template <class T, class A>
orientedpoint<T, A>::orientedpoint(const point<T> &p) {
  this->x = p.x;
  this->y = p.y;
  this->theta = 0.;
}

/*位姿的 + 操作符的重载*/
template <class T, class A>
orientedpoint<T, A> operator+(const orientedpoint<T, A> &p1,
                              const orientedpoint<T, A> &p2) {
  return orientedpoint<T, A>(p1.x + p2.x, p1.y + p2.y, p1.theta + p2.theta);
}

/*位姿的 - 操作符的重载*/
template <class T, class A>
orientedpoint<T, A> operator-(const orientedpoint<T, A> &p1,
                              const orientedpoint<T, A> &p2) {
  return orientedpoint<T, A>(p1.x - p2.x, p1.y - p2.y, p1.theta - p2.theta);
}

/*位姿的 * 操作符的重载  乘以一个数*/
template <class T, class A>
orientedpoint<T, A> operator*(const orientedpoint<T, A> &p, const T &v) {
  return orientedpoint<T, A>(p.x * v, p.y * v, p.theta * v);
}

/*位姿的 * 操作符的重载  乘以一个数*/
template <class T, class A>
orientedpoint<T, A> operator*(const T &v, const orientedpoint<T, A> &p) {
  return orientedpoint<T, A>(p.x * v, p.y * v, p.theta * v);
}
template <class T, class A>
std::ostream &operator<<(std::ostream &os, const orientedpoint<T, A> &p) {
  os << "x:" << p.x << " y:" << p.y << " theta:" << p.theta;
  return os;
}
/*
@desc 两个位姿的差值，算出来P1在以P2为原点的坐标系里面的坐标。
*/
template <class T, class A>
orientedpoint<T, A> absoluteDifference(const orientedpoint<T, A> &p1,
                                       const orientedpoint<T, A> &p2) {
  orientedpoint<T, A> delta = p1 - p2;
  delta.theta = atan2(sin(delta.theta), cos(delta.theta));
  double s = sin(p2.theta), c = cos(p2.theta);
  return orientedpoint<T, A>(c * delta.x + s * delta.y,
                             -s * delta.x + c * delta.y, delta.theta);
}

/*
@desc 两个位姿的和 p2表示增量。该函数表示在P1的位姿上，加上一个P2的增量
*/
template <class T, class A>
orientedpoint<T, A> absoluteSum(const orientedpoint<T, A> &p1,
                                const orientedpoint<T, A> &p2) {
  double s = sin(p1.theta), c = cos(p1.theta);
  return orientedpoint<T, A>(c * p2.x - s * p2.y, s * p2.x + c * p2.y,
                             p2.theta) +
         p1;
}

/*一个位姿和一个位置的和*/
template <class T, class A>
point<T> absoluteSum(const orientedpoint<T, A> &p1, const point<T> &p2) {
  double s = sin(p1.theta), c = cos(p1.theta);
  return point<T>(c * p2.x - s * p2.y, s * p2.x + c * p2.y) + (point<T>)p1;
}

/*点的比较函数 一般用于排序*/
template <class T>
struct pointcomparator {
  bool operator()(const point<T> &a, const point<T> &b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};

/*点构成的角度的比较函数  一般用于排序*/
template <class T>
struct pointradialcomparator {
  point<T> origin;
  bool operator()(const point<T> &a, const point<T> &b) const {
    point<T> delta1 = a - origin;
    point<T> delta2 = b - origin;
    return (atan2(delta1.y, delta1.x) < atan2(delta2.y, delta2.x));
  }
};

/*把两个点的最大的X和最大的Y返回*/
template <class T>
inline point<T> max(const point<T> &p1, const point<T> &p2) {
  point<T> p = p1;
  p.x = p.x > p2.x ? p.x : p2.x;
  p.y = p.y > p2.y ? p.y : p2.y;
  return p;
}

/*把两个点的最小的X和最小的Y返回*/
template <class T>
inline point<T> min(const point<T> &p1, const point<T> &p2) {
  point<T> p = p1;
  p.x = p.x < p2.x ? p.x : p2.x;
  p.y = p.y < p2.y ? p.y : p2.y;
  return p;
}

/*
P1P2这份方向的线性插值
T1表示P1，T2表示P2
T3表示P1和P2之间的一个点
*/
template <class T, class F>
inline point<T> interpolate(const point<T> &p1, const F &t1, const point<T> &p2,
                            const F &t2, const F &t3) {
  F gain = (t3 - t1) / (t2 - t1);
  point<T> p = p1 + (p2 - p1) * gain;
  return p;
}

/*
两个位姿之间的插值
T1.T2.T3的定义如上个函数
*/
template <class T, class A, class F>
inline orientedpoint<T, A>
interpolate(const orientedpoint<T, A> &p1, const F &t1,
            const orientedpoint<T, A> &p2, const F &t2, const F &t3) {
  F gain = (t3 - t1) / (t2 - t1);
  orientedpoint<T, A> p;
  p.x = p1.x + (p2.x - p1.x) * gain;
  p.y = p1.y + (p2.y - p1.y) * gain;
  double s = sin(p1.theta) + sin(p2.theta) * gain,
         c = cos(p1.theta) + cos(p2.theta) * gain;
  p.theta = atan2(s, c);
  return p;
}

/*两个点的欧式距离*/
template <class T>
inline double euclidianDist(const point<T> &p1, const point<T> &p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

/*两个位姿的欧式距离*/
template <class T, class A>
inline double euclidianDist(const orientedpoint<T, A> &p1,
                            const orientedpoint<T, A> &p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class T, class A>
inline double squareDist(const orientedpoint<T, A> &p1,
                         const orientedpoint<T, A> &p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

/*位姿和点的欧式距离*/
template <class T, class A>
inline double euclidianDist(const orientedpoint<T, A> &p1, const point<T> &p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

/*点和位姿的欧式距离*/
template <class T, class A>
inline double euclidianDist(const point<T> &p1, const orientedpoint<T, A> &p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

/*几个不同类型的重定向指令*/
typedef point<int> IntPoint;
typedef point<double> Point;
typedef orientedpoint<double, double> OrientedPoint;

};  // namespace basic

#endif
