#include "message_bus.h"
#include <mutex>
#include <cassert>

namespace Framework {

MessageBus& MessageBus::Instance() {
  static MessageBus instance;
  return instance;
}

}  // namespace Framework

#ifdef _WIN32
#define FRAMEWORK_EXPORT __declspec(dllexport)
#else
#define FRAMEWORK_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {
FRAMEWORK_EXPORT Framework::MessageBus* GetMessageBusInstance() {
  return &Framework::MessageBus::Instance();
}
}

