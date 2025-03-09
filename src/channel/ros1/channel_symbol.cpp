#include "rosnode.h"
extern "C" {
VirtualChannelNode *GetChannelInstance() { return new RosNode(); }
}