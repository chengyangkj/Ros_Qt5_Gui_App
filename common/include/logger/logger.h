#pragma once
enum LogLevel {
  DEBUG,
  INFO,
  WARNING,
  ERROR,
  FATAL

};
static void InitLogger() {
  // el::Configurations defaultConf;
  // defaultConf.setToDefault();
  // //设置最大文件大小
  // defaultConf.setGlobally(el::ConfigurationType::MaxLogFileSize, "1000000");
  // //是否写入文件
  // defaultConf.setGlobally(el::ConfigurationType::ToFile, "true");
  // //是否输出控制台
  // defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
  // // filename
  // defaultConf.setGlobally(el::ConfigurationType::Filename,
  //                         "ros_qt_gui_app.log");
  // //设置配置文件
  // el::Loggers::reconfigureLogger("default", defaultConf);

  // /// 防止Fatal级别日志中断程序
  // el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);

  // /// 选择划分级别的日志
  // el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);

  // /// 设置级别门阀值，修改参数可以控制日志输出
  // el::Loggers::setLoggingLevel(el::Level::Global);
}
#ifndef LOG_INFO
#define LOG_INFO(str)                                                          \
  { std::cout << str << std::endl; }
#endif
#ifndef LOG_ERROR
#define LOG_ERROR(str)                                                         \
  { std::cout << str << std::endl; }
#endif
#ifndef LOG_WARN
#define LOG_WARN(str)                                                          \
  { std::cout << str << std::endl; }
#endif