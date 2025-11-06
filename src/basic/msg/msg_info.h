#pragma once
#include "str_enum.h"

#define SOME_ENUM(OneValue)                                                    \
  OneValue(kOccupancyMap, ) OneValue(kLocalCostMap, )                          \
      OneValue(kGlobalCostMap, ) OneValue(kRobotPose, ) OneValue(kLaserScan, ) \
          OneValue(kLocalPath, ) OneValue(kGlobalPath, ) OneValue(kOdomPose, ) \
              OneValue(kSetNavGoalPose, ) OneValue(kSetRelocPose, )            \
                  OneValue(kSetRobotSpeed, ) OneValue(kBatteryState, ) OneValue(kImage, ) \
                  OneValue(kRobotFootprint, ) OneValue(kOccMap, ) OneValue(kTopologyMap, ) \
                  OneValue(kTopologyMapUpdate, )

DECLARE_ENUM(MsgId, SOME_ENUM)
DEFINE_ENUM(MsgId, SOME_ENUM)

#define MSG_ID_OCCUPANCY_MAP ToString(MsgId::kOccupancyMap)
#define MSG_ID_LOCAL_COST_MAP ToString(MsgId::kLocalCostMap)
#define MSG_ID_GLOBAL_COST_MAP ToString(MsgId::kGlobalCostMap)
#define MSG_ID_ROBOT_POSE ToString(MsgId::kRobotPose)
#define MSG_ID_LASER_SCAN ToString(MsgId::kLaserScan)
#define MSG_ID_LOCAL_PATH ToString(MsgId::kLocalPath)
#define MSG_ID_GLOBAL_PATH ToString(MsgId::kGlobalPath)
#define MSG_ID_ODOM_POSE ToString(MsgId::kOdomPose)
#define MSG_ID_SET_NAV_GOAL_POSE ToString(MsgId::kSetNavGoalPose)
#define MSG_ID_SET_RELOC_POSE ToString(MsgId::kSetRelocPose)
#define MSG_ID_SET_ROBOT_SPEED ToString(MsgId::kSetRobotSpeed)
#define MSG_ID_BATTERY_STATE ToString(MsgId::kBatteryState)
#define MSG_ID_IMAGE ToString(MsgId::kImage)
#define MSG_ID_ROBOT_FOOTPRINT ToString(MsgId::kRobotFootprint)
#define MSG_ID_OCC_MAP ToString(MsgId::kOccMap)
#define MSG_ID_TOPOLOGY_MAP ToString(MsgId::kTopologyMap)
#define MSG_ID_TOPOLOGY_MAP_UPDATE ToString(MsgId::kTopologyMapUpdate)

#define DISPLAY_ROBOT ToString(MsgId::kRobotPose)
#define DISPLAY_MAP ToString(MsgId::kOccupancyMap)
#define DISPLAY_LOCAL_COST_MAP ToString(MsgId::kLocalCostMap)
#define DISPLAY_GLOBAL_COST_MAP ToString(MsgId::kGlobalCostMap)
#define DISPLAY_GLOBAL_PATH ToString(MsgId::kGlobalPath)
#define DISPLAY_LOCAL_PATH ToString(MsgId::kLocalPath)
#define DISPLAY_LASER ToString(MsgId::kLaserScan)
#define DISPLAY_PARTICLE "Particle"
#define DISPLAY_REGION "Region"
#define DISPLAY_TAG "Tag"
#define DISPLAY_GOAL "GoalPose"
#define DISPLAY_TOPOLINE "TopologyLine"
#define DISPLAY_ROBOT_FOOTPRINT ToString(MsgId::kRobotFootprint)
#define DISPLAY_TOPOLOGY_MAP ToString(MsgId::kTopologyMap)
