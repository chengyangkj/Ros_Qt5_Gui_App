/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:06:32
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-26 10:13:06
 * @FilePath: /ros_qt5_gui_app/include/basic/algorithm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ROBOTALGORITHM_H
#define ROBOTALGORITHM_H
#include <math.h>

namespace basic {

// 角度转弧度
inline double deg2rad(double x) { return M_PI * x / 180.0; }
// 弧度转角度
inline double rad2deg(double x) { return 180.0 * x / M_PI; }
} // namespace basic

#endif // ROBOTALGORITHM_H
