#pragma once
#include <memory>
#include "mainwindow/mainwindow.h"

class ApplicationManager {
 public:
  ApplicationManager();
  ~ApplicationManager();
  
  ApplicationManager(const ApplicationManager&) = delete;
  ApplicationManager& operator=(const ApplicationManager&) = delete;
  
  bool Initialize();
  void Shutdown();

 private:
  std::unique_ptr<MainWindow> main_window_;
  bool initialized_{false};
};
