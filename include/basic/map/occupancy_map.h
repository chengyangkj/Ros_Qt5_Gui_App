/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-03-30 15:38:12
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-26 16:44:24
 * @FilePath: /hontai/src/tools/localizationViewer/include/map/occupancy_map.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * 图元坐标系:
 * ++++++x
 * +
 * +
 * y
 * 栅格地图坐标系:
 * +++++++y
 * +
 * +
 * x
 * 世界坐标系:
 * y
 * +
 * +
 * ++++++x
 *
 */
#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H
#include <Eigen/Dense>
#include <iostream>
namespace basic {

class OccupancyMap {
 public:
  double origin_x{0};      // 地图原点x(栅格地图左下角)
  double origin_y{0};      // 地图原点y(栅格地图左下角)
  double origin_theta{0};  // 地图原点theta(栅格地图左下角)
  double resolution{0};    // 地图分辨率
  Eigen::Vector3d origin_pose{0, 0, 0};  // 地图原点位置(栅格地图左下角)
  int rows{0};                           // 行(高)
  int cols{0};                           // 列(宽)
  Eigen::MatrixXi map_data;  // 地图数据,数据的地图已经被上下翻转
 public:
  OccupancyMap() {}
  OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res)
      : rows(rows_),
        cols(cols_),
        origin_pose(origin),
        origin_x(origin[0]),
        origin_y(origin[1]),
        origin_theta(origin[2]),
        resolution(res),
        map_data(rows_,cols) {
        }
      OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res,
               Eigen::MatrixXi data )
      : rows(rows_),
        cols(cols_),
        origin_pose(origin),
        origin_x(origin[0]),
        origin_y(origin[1]),
        origin_theta(origin[2]),
        resolution(res),
        map_data(data) {
        }
  ~OccupancyMap() = default;
  Eigen::MatrixXi GetMapData() { return map_data; }
  Eigen::MatrixXi flip() { return map_data.colwise().reverse(); }
  void SetFlip() { map_data = flip(); }
  auto& operator()(int r, int c) {
     return map_data(r, c); }
  // 输入原始栅格地图数据(地图未翻转)
  void SetMapData(const Eigen::MatrixXi& data) {
    map_data = data;
    map_data = flip();
  }
  int Rows() { return rows; }
  int height() { return rows; }
  int Cols() { return cols; }
  int width() { return cols; }
  /**
   * @description: 输入栅格地图的行与列号，返回该位置的全局坐标
   * @param {int&} c 列号
   * @param {int&} r 行号
   * @param {double&} x x坐标
   * @param {double&} y y坐标
   * @return {*}
   */
  void idx2xy(const int& c, const int& r, double& x, double& y) {
    x = origin_x + c * resolution;
    y = origin_y + r * resolution;
  }
  /**
   * @description: 输入全局坐标，返回栅格地图的行与列号
   * @param {double&} x
   * @param {double&} y
   * @param {int&} c 列号
   * @param {int&} r 行号
   * @return {*}
   */
  void xy2idx(const double& x, const double& y, int& c, int& r) {
    c = round(x - origin_x) / resolution;
    r = round(y - origin_y) / resolution;
  }
  /**
   * @description:输入全局坐标，判断是否在栅格地图内
   * @param {double&} x
   * @param {double&} y
   * @return {*}
   */
  bool inMap(const double& x, const double& y) {
    int c_idx = round((x - origin_x) / resolution);
    int r_idx = round((y - origin_y) / resolution);
    return (c_idx >= 0 && r_idx >= 0 && c_idx < cols && r_idx < rows);
  }
  /**
   * @description:输入行与列号，返回该位置是否在栅格地图内
   * @param {int} r_idx
   * @param {int} c_idx
   * @return {*}
   */
  bool inMap(int r_idx, int c_idx) const {
    return (c_idx >= 0 && r_idx >= 0 && c_idx < cols && r_idx < rows);
  }
  /**
   * @description:输入原始(地图图片)图元坐标,返回世界坐标
   * @return {*}
   */
  void scene2xy(const int& scene_x, const int& scene_y, double& word_x,
                double& word_y) {
    word_x = scene_x * resolution + origin_x;
    word_y = (height() - scene_y) * resolution + origin_y;
  }
  /**
   * @description:
   * 输入世界坐标,返回原始(地图图片)图元坐标。坐标已经进行了上下翻转
   * @param {double&} word_x
   * @param {double&} word_x
   * @param {int&} scene_x
   * @param {int&} scene_y
   * @return {*}
   */
  void xy2scene(const double& word_x, const double& word_y, int& scene_x,
                int& scene_y) {
    scene_x = (word_x - origin_x) / resolution;
    scene_y = height() - (word_y - origin_y) / resolution;
  }
};

}  // namespace basic

#endif