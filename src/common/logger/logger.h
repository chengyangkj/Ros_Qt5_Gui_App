#pragma once
#include <iostream>
#include <sstream>
#include "macros.h"

#ifdef ERROR
#undef ERROR
#endif

enum class LogLevel { kInfo,
                      kError,
                      kWarn };
#define LOG_INFO(message)                        \
  do {                                           \
    std::stringstream ss;                        \
    ss << message;                               \
    Logger::Instance()->Log(LogLevel::kInfo, ss, __FILE__, __LINE__); \
  } while (0);

#define LOG_ERROR(message)                        \
  do {                                            \
    std::stringstream ss;                         \
    ss << message;                                \
    Logger::Instance()->Log(LogLevel::kError, ss, __FILE__, __LINE__); \
  } while (0);

#define LOG_WARN(message)                        \
  do {                                           \
    std::stringstream ss;                        \
    ss << message;                               \
    Logger::Instance()->Log(LogLevel::kWarn, ss, __FILE__, __LINE__); \
  } while (0);

class Logger {
 public:

  ~Logger();
  void Log(LogLevel level, const std::stringstream& message, const char* file = nullptr, int line = 0);

  DEFINE_SINGLETON(Logger)
};
