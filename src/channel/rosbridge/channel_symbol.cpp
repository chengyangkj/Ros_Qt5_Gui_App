#include "rosbridge_comm.h"
extern "C" {
VirtualChannelNode *GetChannelInstance() { return new RosbridgeComm(); }
}

