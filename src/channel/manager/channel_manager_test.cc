#include "channel_manager.h"
#include "gtest/gtest.h"

TEST(ChannelManager, OpenDL) {
  ChannelManager manager;
  // EXPECT_TRUE(manager.OpenChannelAuto());
  // manager.RegisterOnDataCallback(
  //     std::move([=](const MsgId &id, const std::any &data) {

  //     }));
  // manager.SendMessage(MsgId::kSetNavGoalPose,
  //                     basic::RobotPose(0.1, 0.001, 0.1));
}