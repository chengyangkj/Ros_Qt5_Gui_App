#pragma once
#include "str_enum.h"

#define SOME_ENUM(OneValue)                                                    \
  OneValue(kOccupancyMap, ) OneValue(kLocalCostMap, )                          \
      OneValue(kGlobalCostMap, ) OneValue(kRobotPose, ) OneValue(kLaserScan, ) \
          OneValue(kLocalPath, ) OneValue(kGlobalPath, ) OneValue(kOdomPose, ) \
              OneValue(kSetNavGoalPose, ) OneValue(kSetRelocPose, )            \
                  OneValue(kSetRobotSpeed, ) OneValue(kBatteryState, ) OneValue(kImage, )

DECLARE_ENUM(MsgId, SOME_ENUM)
DEFINE_ENUM(MsgId, SOME_ENUM)
