#pragma once
#include <iostream>
#include <sstream>
enum class LogLevel { INFO,
                      ERROR,
                      WARN };
#define LOG_INFO(message)                        \
  do {                                           \
    std::stringstream ss;                        \
    ss << message;                               \
    Logger::Instance()->Log(LogLevel::INFO, ss); \
  } while (0);

#define LOG_ERROR(message)                        \
  do {                                            \
    std::stringstream ss;                         \
    ss << message;                                \
    Logger::Instance()->Log(LogLevel::ERROR, ss); \
  } while (0);

#define LOG_WARN(message)                        \
  do {                                           \
    std::stringstream ss;                        \
    ss << message;                               \
    Logger::Instance()->Log(LogLevel::WARN, ss); \
  } while (0);

class Logger {
 public:
  static Logger* Instance();
  Logger();
  ~Logger();
  void Log(LogLevel level, const std::stringstream& message);
};
