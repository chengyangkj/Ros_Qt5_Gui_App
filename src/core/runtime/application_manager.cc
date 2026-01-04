#include "application_manager.h"
#include "common/logger/logger.h"

ApplicationManager::ApplicationManager() = default;

ApplicationManager::~ApplicationManager() {
  Shutdown();
}

bool ApplicationManager::Initialize() {
  if (initialized_) {
    return true;
  }
  
  try {
    main_window_ = std::make_unique<MainWindow>();
    main_window_->show();
    initialized_ = true;
    LOG_INFO("Application initialized successfully");
    return true;
  } catch (const std::exception& e) {
    LOG_ERROR("Failed to initialize application: " << e.what());
    return false;
  }
}

void ApplicationManager::Shutdown() {
  if (!initialized_) {
    return;
  }
  
  main_window_.reset();
  initialized_ = false;
  LOG_INFO("Application shutdown completed");
}
