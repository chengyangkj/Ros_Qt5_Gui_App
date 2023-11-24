#include "rclcomm.h"
extern "C" {
VirtualChannelNode *GetChannelInstance() { return new rclcomm(); }
}