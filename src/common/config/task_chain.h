#pragma once
#include <nlohmann/json.hpp>
#include "topology_map.h"
struct TaskChain {
  std::vector<TopologyMap::PointInfo> points;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TaskChain, points);
