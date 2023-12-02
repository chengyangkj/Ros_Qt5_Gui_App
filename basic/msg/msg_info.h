#pragma once
namespace Msg {
enum MsgId {
  kOccupancyMap = 0,
  kLocalCostMap = 1,
  kGlobalCostMap = 2,
  kRobotPose = 3,
  kLaserScan = 4,
  kLocalPath,
  kGlobalPath,
  kOdomPose,
  kSetNavGoalPose,
  kSetRelocPose,
};
} // namespace Msg
