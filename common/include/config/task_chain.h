#pragma once
#include "json_struct/json_struct.h"
#include "topology_map.h"
struct TaskChain {
  std::vector<TopologyMap::PointInfo> points;
  JS_OBJ(points);
};
