/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-30 15:38:12
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-09-27 15:07:39
 * @FilePath: ////include/map/occupancy_map.h
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
#ifndef COST_MAP_H
#define COST_MAP_H
#include <Eigen/Dense>

namespace basic {

class CostMap {
public:
  double origin_x{0};     // 地图原点x(栅格地图左下角)
  double origin_y{0};     // 地图原点y(栅格地图左下角)
  double origin_theta{0}; // 地图原点theta(栅格地图左下角)
  double resolution{0};   // 地图分辨率
  Eigen::Vector3d origin_pose{0, 0, 0}; // 地图原点位置(栅格地图左下角)
  int rows{0};                          // 行(高)
  int cols{0};                          // 列(宽)
  Eigen::MatrixXi map_data; // 地图数据,数据的地图已经被上下翻转
public:
  CostMap() {}
  CostMap(int rows_, int cols_, Eigen::Vector3d origin, double res)
      : rows(rows_), cols(cols_), origin_pose(origin), origin_x(origin[0]),
        origin_y(origin[1]), origin_theta(origin[2]), resolution(res),
        map_data(rows_, cols_) {}
  CostMap(int rows_, int cols_, Eigen::Vector3d origin, double res,
          Eigen::MatrixXi data)
      : rows(rows_), cols(cols_), origin_pose(origin), origin_x(origin[0]),
        origin_y(origin[1]), origin_theta(origin[2]), resolution(res),
        map_data(data) {}
  ~CostMap() = default;
  Eigen::MatrixXi GetMapData() { return map_data; }
  // 沿X上下翻转
  Eigen::MatrixXi flip() { return map_data.colwise().reverse(); }
  void SetFlip() { map_data = flip(); }
  auto &operator()(int r, int c) { return map_data(r, c); }

  // 输入原始栅格地图数据(地图未翻转)
  void SetMapData(const Eigen::MatrixXi &data) {
    map_data = data;
    // 翻转栅格地图坐标系
    map_data = flip();
  }
  /**
   * @description: 获取带rgba颜色值的代价地图
   * @return {*}
   */
  Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic>
  GetColorMapData() {
    Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic> res =
        Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic>(
            map_data.rows(), map_data.cols());
    for (int x = 0; x < map_data.rows(); x++) {
      for (int y = 0; y < map_data.cols(); y++) {
        // 计算像素值
        Eigen::Vector4i color_rgba;
        int data = map_data(x, y);
        if (data >= 100) {
          color_rgba = Eigen::Vector4i(0xff, 0x00, 0xff, 50);

        } else if (data >= 90 && data < 100) {
          color_rgba = Eigen::Vector4i(0x66, 0xff, 0xff, 50);

        } else if (data >= 70 && data <= 90) {
          color_rgba = Eigen::Vector4i(0xff, 0x00, 0x33, 50);

        } else if (data >= 60 && data <= 70) {
          color_rgba = Eigen::Vector4i(0xbe, 0x28, 0x1a, 50);

        } else if (data >= 50 && data < 60) {
          color_rgba = Eigen::Vector4i(0xBE, 0x1F, 0x58, 50);

        } else if (data >= 40 && data < 50) {
          color_rgba = Eigen::Vector4i(0xBE, 0x25, 0x76, 50);

        } else if (data >= 30 && data < 40) {
          color_rgba = Eigen::Vector4i(0xBE, 0x2A, 0x99, 50);

        } else if (data >= 20 && data < 30) {
          color_rgba = Eigen::Vector4i(0xBE, 0x35, 0xB3, 50);

        } else if (data >= 10 && data < 20) {
          color_rgba = Eigen::Vector4i(0xB0, 0x3C, 0xbE, 50);

        } else {
          // 其他 透明
          color_rgba = Eigen::Vector4i(0, 0, 0, 0);
        }
        res(x, y) = color_rgba;
      }
    }
    return res;
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
  void idx2xy(const int &c, const int &r, double &x, double &y) {
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
  void xy2idx(const double &x, const double &y, int &c, int &r) {
    c = round(x - origin_x) / resolution;
    r = round(y - origin_y) / resolution;
  }
  /**
   * @description:输入全局坐标，判断是否在栅格地图内
   * @param {double&} x
   * @param {double&} y
   * @return {*}
   */
  bool inMap(const double &x, const double &y) {
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
  void scene2xy(const int &scene_x, const int &scene_y, double &word_x,
                double &word_y) {
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
  void xy2scene(const double &word_x, const double &word_y, int &scene_x,
                int &scene_y) {
    scene_x = (word_x - origin_x) / resolution;
    scene_y = height() - (word_y - origin_y) / resolution;
  }
};

} // namespace basic
#endif