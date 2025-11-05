#pragma once
#include <functional>

#ifdef QT_CORE_LIB
#include <QCoreApplication>
#include <QThread>
#include <QTimer>
#include "logger/logger.h"
#endif

namespace Framework {
namespace detail {

class ThreadSafeCallbackExecutor {
 public:
  using ExecuteFunction = std::function<void()>;
  
  static void Execute(ExecuteFunction func) {
    #ifdef QT_CORE_LIB
    QCoreApplication* app = QCoreApplication::instance();
    if (app && QThread::currentThread() != app->thread()) {
    //   LOG_INFO("[ThreadSafeCallbackExecutor] Scheduling callback to main thread");
      QTimer::singleShot(0, app, func);
      return;
    }
    #endif
    // LOG_INFO("[ThreadSafeCallbackExecutor] Executing callback directly in current thread");
    func();
  }
};

}  // namespace detail
}  // namespace Framework

