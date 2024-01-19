#include "application_manager.h"
#include "logger/logger.h"
ApplicationManager::ApplicationManager(/* args */) {
  InitLogger();
  main_window.show();
}

ApplicationManager::~ApplicationManager() {}
