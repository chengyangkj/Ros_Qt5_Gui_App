#include "config/config_manager.h"
#include <gtest/gtest.h>

TEST(TopologyMapTest, ReadAndWriteMap) {
  TopologyMap map;
  map.map_name = "test";
  map.points.push_back(TopologyMap::PointInfo{
      .x = 1.11, .y = 2.22, .theta = 3.33, .name = "test1"});
  map.points.push_back(TopologyMap::PointInfo{
      .x = 2.11, .y = 3.22, .theta = 4.33, .name = "test2"});
  EXPECT_TRUE(Config::ConfigManager::Instacnce()->WriteTopologyMap(
      "./test_map.json", map));

  TopologyMap map_read;
  EXPECT_TRUE(Config::ConfigManager::Instacnce()->ReadTopologyMap(
      "./test_map.json", map_read));
  EXPECT_EQ(map_read.points.size(), map.points.size());
  for (int i = 0; i < map_read.points.size(); i++) {
    EXPECT_EQ(map_read.points[i].x, map.points[i].x);
    EXPECT_EQ(map_read.points[i].y, map.points[i].y);
    EXPECT_EQ(map_read.points[i].theta, map.points[i].theta);
    EXPECT_EQ(map_read.points[i].name, map.points[i].name);
    EXPECT_EQ(map_read.points[i].type, map.points[i].type);
  }
}