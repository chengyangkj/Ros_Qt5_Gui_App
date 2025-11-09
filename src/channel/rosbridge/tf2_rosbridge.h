#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include "point_type.h"

namespace rosbridge2cpp {

struct TransformData {
  double x;
  double y;
  double theta;
  std::string parent_frame;
};

class TF2Rosbridge {
 public:
  TF2Rosbridge() = default;
  ~TF2Rosbridge() = default;

  void UpdateTF(const std::unordered_map<std::string, TransformData> &tf_cache);
  
  basic::RobotPose LookUpForTransform(const std::string &from, const std::string &to);

 private:
  std::string NormalizeFrame(const std::string &frame);
  std::vector<std::string> ShortestPath(const std::string &from, const std::string &to);

  std::unordered_map<std::string, std::unordered_set<std::string>> adj_;
  std::unordered_map<std::string, std::unordered_map<std::string, basic::RobotPose>> adj_transform_;
};

}  // namespace rosbridge2cpp

