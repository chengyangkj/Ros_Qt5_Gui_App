#include "logger/logger.h"

#include "logger/easylogging++.h"
INITIALIZE_EASYLOGGINGPP
Logger* Logger::Instance() {
  static Logger logger;
  return &logger;
}
Logger::Logger() {
  int argc;
  char** argv;
  START_EASYLOGGINGPP(argc, argv);
  el::Configurations defaultConf;
  defaultConf.setToDefault();
  //设置最大文件大小
  defaultConf.setGlobally(el::ConfigurationType::MaxLogFileSize, "100000000");
  //是否写入文件
  defaultConf.setGlobally(el::ConfigurationType::ToFile, "true");
  //是否输出控制台
  defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
  // filename
  defaultConf.setGlobally(el::ConfigurationType::Filename,
                          "ros_qt_gui_app.log");
  defaultConf.setGlobally(el::ConfigurationType::Format, "[%datetime][%level] %msg");
  //设置配置文件
  el::Loggers::reconfigureLogger("default", defaultConf);

  /// 防止Fatal级别日志中断程序
  el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
}
Logger::~Logger() {}
void Logger::Log(LogLevel level, const std::stringstream& message) {
  switch (level) {
    case LogLevel::INFO:
      std::cout << "[INFO] " << message.str() << std::endl;
      LOG(INFO) << message.str();
      break;
    case LogLevel::ERROR:
      std::cerr << "[ERROR] " << message.str() << std::endl;
      LOG(ERROR) << message.str();
      break;
    case LogLevel::WARN:
      std::cerr << "[WARN] " << message.str() << std::endl;
      LOG(WARNING) << message.str();
      break;
  }
}