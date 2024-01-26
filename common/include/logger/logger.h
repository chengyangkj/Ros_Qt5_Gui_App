#pragma once
#include <iostream>
#include <sstream>
enum class LogLevel { INFO,
                      ERROR,
                      WARN };

class Logger {
 public:
  static void log(LogLevel level, const std::stringstream &message) {
    switch (level) {
      case LogLevel::INFO:
        std::cout << "[INFO] " << message.str() << std::endl;
        break;
      case LogLevel::ERROR:
        std::cerr << "[ERROR] " << message.str() << std::endl;
        break;
      case LogLevel::WARN:
        std::cerr << "[WARN] " << message.str() << std::endl;
        break;
    }
  }
};

#define LOG_INFO(message)            \
  do {                               \
    std::stringstream ss;            \
    ss << message;                   \
    Logger::log(LogLevel::INFO, ss); \
  } while (0);

#define LOG_ERROR(message)            \
  do {                                \
    std::stringstream ss;             \
    ss << message;                    \
    Logger::log(LogLevel::ERROR, ss); \
  } while (0);

#define LOG_WARN(message)            \
  do {                               \
    std::stringstream ss;            \
    ss << message;                   \
    Logger::log(LogLevel::WARN, ss); \
  } while (0);
