#include "logger/logger.h"

#include "logger/easylogging++.h"
INITIALIZE_EASYLOGGINGPP
Logger* Logger::Instance() {
  static Logger logger;
  return &logger;
}
Logger::Logger() {
  int argc = 0;
  char** argv = nullptr;
  START_EASYLOGGINGPP(argc, argv);
  el::Configurations defaultConf;
  defaultConf.setToDefault();
  // 设置最大文件大小
  defaultConf.setGlobally(el::ConfigurationType::MaxLogFileSize, "100000000");
  // 是否写入文件
  defaultConf.setGlobally(el::ConfigurationType::ToFile, "true");
  // 是否输出控制台
  defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "true");
  // filename
  defaultConf.setGlobally(el::ConfigurationType::Filename,
                          "ros_qt_gui_app.log");
  defaultConf.setGlobally(el::ConfigurationType::Format, "[%datetime][%level] %msg");
  // 设置配置文件
  el::Loggers::reconfigureLogger("default", defaultConf);

  /// 防止Fatal级别日志中断程序
  el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
}
Logger::~Logger() {}
void Logger::Log(LogLevel level, const std::stringstream& message, const char* file, int line) {
  switch (level) {
    case LogLevel::INFO:
      LOG(INFO) << "[" << file << ":" << line << "] " << message.str();
      break;
    case LogLevel::ERROR:
      LOG(ERROR) << "[" << file << ":" << line << "] " << message.str();
      break;
    case LogLevel::WARN:
      LOG(WARNING) << "[" << file << ":" << line << "] " << message.str();
      break;
  }
}