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
    Logger::Instance()->Log(LogLevel::INFO, ss, __FILE__, __LINE__); \
  } while (0);

#define LOG_ERROR(message)                        \
  do {                                            \
    std::stringstream ss;                         \
    ss << message;                                \
    Logger::Instance()->Log(LogLevel::ERROR, ss, __FILE__, __LINE__); \
  } while (0);

#define LOG_WARN(message)                        \
  do {                                           \
    std::stringstream ss;                        \
    ss << message;                               \
    Logger::Instance()->Log(LogLevel::WARN, ss, __FILE__, __LINE__); \
  } while (0);

class Logger {
 public:
  static Logger* Instance();
  Logger();
  ~Logger();
  void Log(LogLevel level, const std::stringstream& message, const char* file = nullptr, int line = 0);
};
