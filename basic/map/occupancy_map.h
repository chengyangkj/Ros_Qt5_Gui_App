/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-30 15:38:12
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-10 15:23:33
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
#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "logger/logger.h"

#define OCC_GRID_UNKNOWN 0     //未知領域
#define OCC_GRID_FREE 0        //free
#define OCC_GRID_OCCUPIED 100  //占有領域

namespace basic {
struct MapConfig {
  std::string image = "./";
  double resolution = 0.1;
  std::vector<double> origin;
  int negate{0};
  double occupied_thresh{0.25};
  double free_thresh{0.65};
  MapConfig() {
    origin.resize(3);
  }
  MapConfig(std::vector<double> ori, double res) {
    origin = ori;
    resolution = res;
  }
  void Load(const std::string &filename) {
    std::ifstream file(filename);
    origin.clear();
    if (file.is_open()) {
      std::string line;
      while (getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, ':')) {
          std::string value;
          if (std::getline(iss, value)) {
            // 移除值前后的空格
            value = value.substr(value.find_first_not_of(" "), value.find_last_not_of(" ") + 1);
            std::cout << "value:" << value << std::endl;

            // 根据键解析对应的值
            if (key == "image") {
              image = value;
            } else if (key == "resolution") {
              resolution = std::stod(value);
            } else if (key == "origin") {
              std::istringstream issOrigin(value);
              std::string originValue;
              while (std::getline(issOrigin, originValue, ',')) {
                // 移除值前后的空格
                originValue = originValue.substr(originValue.find_first_not_of(" "), originValue.find_last_not_of(" ") + 1);
                std::cout << "origin value:" << originValue << std::endl;
                origin.push_back(std::stod(originValue));
              }
            } else if (key == "negate") {
              negate = std::stoi(value);
            } else if (key == "occupied_thresh") {
              occupied_thresh = std::stod(value);
            } else if (key == "free_thresh") {
              free_thresh = std::stod(value);
            }
          }
        }
      }
      LOG_INFO("Successfully loaded params from ");
      LOG_INFO("image: " << image);
      LOG_INFO("resolution: " << resolution);
      LOG_INFO("origin: ");
      for (auto &o : origin) {
        LOG_INFO(o << " ");
      }
      LOG_INFO("negate: " << negate);
      LOG_INFO("occupied_thresh: " << occupied_thresh);
      LOG_INFO("free_thresh: " << free_thresh);

      file.close();
    } else {
      LOG_INFO("无法打开文件 " << filename);
    }
  }
  void Save(const std::string &filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << "image: " << image << std::endl;
      file << "resolution: " << resolution << std::endl;
      file << "origin: " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
      file << "negate: " << negate << std::endl;
      file << "occupied_thresh: " << occupied_thresh << std::endl;
      file << "free_thresh: " << free_thresh << std::endl;
      file.close();
      LOG_INFO("配置已成功写入到文件 " << filename);
    } else {
      LOG_INFO("无法打开文件 " << filename);
    }
  }
};
class OccupancyMap {
 public:
  MapConfig map_config;
  int rows{0};               // 行(高)
  int cols{0};               // 列(宽)
  Eigen::MatrixXi map_data;  // 地图数据,数据的地图已经被上下翻转
 public:
  OccupancyMap() {}
  OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res)
      : map_config({origin[0], origin[1], origin[2]}, res), rows(rows_), cols(cols_), map_data(rows_, cols) {}
  OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res,
               Eigen::MatrixXi data)
      : map_config({origin[0], origin[1], origin[2]}, res), rows(rows_), cols(cols_), map_data(data) {}
  ~OccupancyMap() = default;
  Eigen::MatrixXi GetMapData() { return map_data; }
  Eigen::MatrixXi flip() { return map_data.colwise().reverse(); }
  void SetFlip() { map_data = flip(); }
  auto &operator()(int r, int c) { return map_data(r, c); }
  // 输入原始栅格地图数据(地图未翻转)
  void SetMapData(const Eigen::MatrixXi &data) {
    map_data = data;
    map_data = flip();
  }
  int Rows() { return rows; }
  int Cols() { return cols; }
  //宽高地图坐标系下的长度
  int width() { return cols; }
  int height() { return rows; }
  //宽map坐标系下的长度
  int widthMap() { return cols * map_config.resolution; }
  int heightMap() { return rows * map_config.resolution; }
  /**
   * @description: 输入栅格地图的行与列号，返回该位置的全局坐标
   * @param {int&} c 列号
   * @param {int&} r 行号
   * @param {double&} x x坐标
   * @param {double&} y y坐标
   * @return {*}
   */
  void idx2xy(const int &c, const int &r, double &x, double &y) {
    x = map_config.origin[0] + c * map_config.resolution;
    y = map_config.origin[1] + r * map_config.resolution;
  }
  /**
   * @description: 输入全局坐标，返回栅格地图的行与列号
   * @param {double&} x
   * @param {double&} y
   * @param {int&} c 列号map_y
   * @param {int&} r 行号
   * @return {*}
   */
  void xy2idx(const double &x, const double &y, int &c, int &r) {
    c = round(x - map_config.origin[0]) / map_config.resolution;
    r = round(y - map_config.origin[1]) / map_config.resolution;
  }
  /**
   * @description:输入全局坐标，判断是否在栅格地图内
   * @param {double&} x
   * @param {double&} y
   * @return {*}
   */
  bool inMap(const double &x, const double &y) {
    int c_idx = round((x - map_config.origin[0]) / map_config.resolution);
    int r_idx = round((y - map_config.origin[1]) / map_config.resolution);
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
  void ScenePose2xy(const double &scene_x, const double &scene_y,
                    double &word_x, double &word_y) {
    word_x = scene_x * map_config.resolution + map_config.origin[0];
    word_y = (height() - scene_y) * map_config.resolution + map_config.origin[1];
  }
  /**
   * @description:输入栅格坐标,返回世界坐标
   * @return {*}
   */
  void OccPose2xy(const double &scene_x, const double &scene_y,
                  double &word_x, double &word_y) {
    word_y = scene_x * map_config.resolution + map_config.origin[0];
    word_x = (height() - scene_y) * map_config.resolution + map_config.origin[1];
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
  void xy2ScenePose(const double &word_x, const double &word_y, double &scene_x,
                    double &scene_y) {
    scene_x = (word_x - map_config.origin[0]) / map_config.resolution;
    scene_y = height() - (word_y - map_config.origin[1]) / map_config.resolution;
  }
  /**
   * @description:
   * 输入世界坐标,返回栅格坐标。坐标已经进行了上下翻转
   * @param {double&} word_x
   * @param {double&} word_x
   * @param {int&} scene_x
   * @param {int&} scene_y
   * @return {*}
   */
  void xy2OccPose(const double &word_x, const double &word_y, double &scene_x,
                  double &scene_y) {
    scene_y = (word_x - map_config.origin[0]) / map_config.resolution;
    scene_x = height() - (word_y - map_config.origin[1]) / map_config.resolution;
  }
  /**
   * @description: 获取带rgba颜色值的代价地图
   * @return {*}
   */
  Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic>
  GetCostMapData() {
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
  //保存地图到路径
  void Save(std::string map_name) {
    std::string mapdatafile = map_name + ".pgm";
    printf("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE *out = fopen(mapdatafile.c_str(), "w");
    if (!out) {
      printf("Couldn't save map file to %s", mapdatafile.c_str());
      return;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map_config.resolution, width(), height());
    for (unsigned int y = 0; y < height(); y++) {
      for (unsigned int x = 0; x < width(); x++) {
        // unsigned int i = x + (height() - y - 1) * map->info.width;
        if (map_data(y, x) >= 0 && map_data(y, x) <= map_config.free_thresh) {  // [0,free)
          fputc(254, out);
        } else if (map_data(y, x) >= map_config.occupied_thresh) {  // (occ,255]
          fputc(000, out);
        } else {  //occ [0.25,0.65]
          fputc(205, out);
        }
      }
    }

    fclose(out);

    std::string mapmetadatafile = map_name + ".yaml";
    printf("Writing map occupancy data to %s", mapmetadatafile.c_str());

    /*
map_config.resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */
    map_config.image = map_name;
    map_config.Save(mapmetadatafile);
  }
  void Load(const std::string &map_path, const std::string &yaml_path) {
    //解析yaml
    std::ifstream file(map_path);
    if (file.is_open()) {
      std::string line;
      int width, height, maxVal;

      // 读取 PGM 文件头信息
      getline(file, line);      // 第一行是 "P5"，表示文件类型
      getline(file, line);      // 第二行是注释，可以忽略
      file >> width >> height;  // 第三行是宽度和高度
      file >> maxVal;           // 第四行是最大像素值
      //赋值行与列
      rows = height;
      cols = width;
      LOG_INFO("reade from pgm width:" << width << " height:" << height << " maxVal:" << maxVal);
      map_data = Eigen::MatrixXi(height, width);
      // 读取地图数据
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          int8_t pixel = -2;
          int8_t map_cell;
          file >> pixel;
          if (pixel == 0) {
            map_cell = OCC_GRID_OCCUPIED;  //占用
          } else if (pixel == 205) {
            map_cell = OCC_GRID_UNKNOWN;
          } else if (pixel == 254) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          map_data(i, j) = map_cell;
        }
      }
      LOG_INFO("load map over");
      file.close();
    } else {
      LOG_INFO("无法打开地图文件 " << map_path);
    }
    //解析yaml
    map_config.Load(yaml_path);
  }
};

}  // namespace basic

#endif