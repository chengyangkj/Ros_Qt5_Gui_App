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
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "logger/logger.h"
#include "yaml-cpp/yaml.h"
#include <SDL/SDL_image.h>
#define OCC_GRID_UNKNOWN -1    //未知領域
#define OCC_GRID_FREE 0        //free
#define OCC_GRID_OCCUPIED 100  //占有領域
// We use SDL_image to load the image from disk
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace basic {
struct MapConfig {
  enum MapMode{
    TRINARY,
    SCALE,
    RAW
  };
  std::string image = "./";
  double resolution = 0.1;
  std::vector<double> origin;
  int negate{0};
  double occupied_thresh{0.25};
  double free_thresh{0.65};
  MapMode mode;
  MapConfig() {
    origin.resize(3);
  }
  MapConfig(std::vector<double> ori, double res) {
    origin = ori;
    resolution = res;
  }
  MapConfig(const MapConfig &other) = default;
  MapConfig &operator=(const MapConfig &other) = default;
  MapConfig(MapConfig &&other) noexcept = default;
  MapConfig &operator=(MapConfig &&other) noexcept = default;
  bool Load(const std::string &filename) {
    std::ifstream fin(filename.c_str());
    if (fin.fail()) {
      LOG_ERROR("Map_server could not open " << filename.c_str());
      return false;
    }
    YAML::Node doc = YAML::Load(fin);
    try {
      resolution = doc["resolution"].as<double>();
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain a resolution tag or it is invalid.");
      return false;
    }
    try {
      negate = doc["negate"].as<int>();
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain a negate tag or it is invalid.");
      return false;
    }
    try {
      occupied_thresh = doc["occupied_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
      return false;
    }
    try {
      free_thresh = doc["free_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain a free_thresh tag or it is invalid.");
      return false;
    }

     try {
       std::string modeS = "";
       modeS=doc["mode"].as<std::string>();

       if (modeS == "trinary")
         mode = TRINARY;
       else if (modeS == "scale")
         mode = SCALE;
       else if (modeS == "raw")
         mode = RAW;
       else {
         LOG_ERROR("Invalid mode tag "<< modeS);
         return false;
       }
     } catch (YAML::Exception &) {
       LOG_INFO("The map does not contain a mode tag or it is invalid... assuming Trinary");
       mode = TRINARY;
     }
    try {
      origin = doc["origin"].as<std::vector<double>>();
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain an origin tag or it is invalid.");
      return false;
    }
    try {
      image = doc["image"].as<std::string>();
      // TODO: make this path-handling more robust
      if (image.size() == 0) {
        LOG_ERROR("The image tag cannot be an empty string.");
        return false;
      }

      boost::filesystem::path mapfpath(image);
      if (!mapfpath.is_absolute()) {
        boost::filesystem::path dir(filename);
        dir = dir.parent_path();
        mapfpath = dir / mapfpath;
        image = mapfpath.string();
      }
    } catch (YAML::InvalidScalar &) {
      LOG_ERROR("The map does not contain an image tag or it is invalid.");
      return false;
    }
    return true;
  }
  void Save(const std::string &filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << "image: " << image << std::endl;
      file << "resolution: " << resolution << std::endl;
      file << "origin: [" << origin[0] << ", " << origin[1] << ", " << origin[2] << "]" << std::endl;
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
  OccupancyMap(const OccupancyMap &other) = default;
  OccupancyMap &operator=(const OccupancyMap &other) = default;
  OccupancyMap(OccupancyMap &&other) noexcept = default;
  OccupancyMap &operator=(OccupancyMap &&other) noexcept = default;
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
    boost::filesystem::path filepath(map_name);
    std::string map_name_rel = filepath.stem().string();
    map_config.image = "./" + map_name_rel + ".pgm";
    map_config.Save(mapmetadatafile);
  }
  bool Load(const std::string &yaml_path) {
    if (!map_config.Load(yaml_path)) {
      return false;
    }

    SDL_Surface* img = IMG_Load(map_config.image.c_str());
    if (!img) {
      LOG_ERROR("failed to open image file \"" << map_config.image << "\": " << IMG_GetError());
      return false;
    }

    int height = img->h;
    int width = img->w;
    rows = height;
    cols = width;
    LOG_INFO("read from image width:" << width << " height:" << height);
    
    map_data = Eigen::MatrixXi(height, width);
    
    int rowstride = img->pitch;
    int n_channels = img->format->BytesPerPixel;
    bool has_alpha = (img->format->Amask != 0);
    
    unsigned char* pixels = static_cast<unsigned char*>(img->pixels);
    
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        unsigned char* p = pixels + y * rowstride + x * n_channels;
        
        std::vector<unsigned char> channels;
        for (int k = 0; k < n_channels; k++) {
          channels.push_back(p[k]);
        }
        
        bool alpha_channel_exists = (n_channels > 1 && has_alpha);
        unsigned char alpha_value = alpha_channel_exists ? p[n_channels - 1] : 255;
        bool is_transparent = alpha_channel_exists && (alpha_value < 255);
        
        if (map_config.mode == MapConfig::MapMode::TRINARY && alpha_channel_exists) {
          channels.push_back(255 - alpha_value);
        }
        
        double sum = 0.0;
        for (auto c : channels) {
          sum += c;
        }
        double shade = sum / channels.size() / 255.0;
        
        double occ = (map_config.negate ? shade : 1.0 - shade);
        
        int map_cell;
        
        switch (map_config.mode) {
          case MapConfig::MapMode::TRINARY: {
            if (occ > map_config.occupied_thresh) {
              map_cell = OCC_GRID_OCCUPIED;
            } else if (occ < map_config.free_thresh) {
              map_cell = OCC_GRID_FREE;
            } else {
              map_cell = OCC_GRID_UNKNOWN;
            }
            break;
          }
          
          case MapConfig::MapMode::SCALE: {
            if (is_transparent) {
              map_cell = OCC_GRID_UNKNOWN;
            } else if (occ > map_config.occupied_thresh) {
              map_cell = OCC_GRID_OCCUPIED;
            } else if (occ < map_config.free_thresh) {
              map_cell = OCC_GRID_FREE;
            } else {
              double ratio = (occ - map_config.free_thresh) / 
                            (map_config.occupied_thresh - map_config.free_thresh);
              map_cell = static_cast<int>(std::rint(ratio * 98.0 + 1.0));
            }
            break;
          }
          
          case MapConfig::MapMode::RAW: {
            double occ_percent = std::round(shade * 255.0);
            if (OCC_GRID_FREE <= occ_percent && occ_percent <= OCC_GRID_OCCUPIED) {
              map_cell = static_cast<int>(occ_percent);
            } else {
              map_cell = OCC_GRID_UNKNOWN;
            }
            break;
          }
          
          default: {
            LOG_ERROR("Invalid map mode");
            SDL_FreeSurface(img);
            return false;
          }
        }
        
        map_data(height - y - 1, x) = map_cell;
      }
    }

    SDL_FreeSurface(img);
    LOG_INFO("Map loaded successfully: " << width << " X " << height 
             << " @ " << map_config.resolution << " m/cell");
    SetFlip();
    return true;
  }
};

}  // namespace basic

#endif