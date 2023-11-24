#pragma once
#include <ros/ros.h>

#include "channel/base/virtual_channel_node.h"
class rosnode : public VirtualChannelNode {
  Q_OBJECT
private:
  /* data */
public:
  rosnode(/* args */);
  ~rosnode() override;

  void run() override;
public slots:
  void PubRelocPose(Eigen::Vector3f pose) override;
  void PubNavGoal(Eigen::Vector3f pose) override;

private:
  int init_argc;
  char **init_argv;
};