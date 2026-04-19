#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace basic {

struct DiagnosticComponentState {
  int level = 0;
  std::string message;
  std::map<std::string, std::string> key_values;
  int64_t last_update_ms = 0;
};

struct DiagnosticSnapshot {
  std::map<std::string, std::map<std::string, DiagnosticComponentState>> hardware;
};

}  // namespace basic
