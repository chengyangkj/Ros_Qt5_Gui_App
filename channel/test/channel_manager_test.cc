#include "channel_manager.h"
#include "gtest/gtest.h"

TEST(ChannelManager, OpenDL) {
  ChannelManager manager;
  EXPECT_TRUE(manager.OpenChannel("./lib/libchannel_ros2.so"));
  manager.RegisterOnDataCallback(
      std::move([=](const MsgId &id, const std::any &data) {
        
      }));
}