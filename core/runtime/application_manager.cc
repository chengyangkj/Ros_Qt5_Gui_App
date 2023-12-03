#include "application_manager.h"
#include "logger/logger.h"
INITIALIZE_EASYLOGGINGPP; // logger
ApplicationManager::ApplicationManager(/* args */) {
  InitLogger();
  main_window.show();
}

ApplicationManager::~ApplicationManager() {}
