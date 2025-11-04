#pragma once
#include <QWidget>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>
#include <map>
#include <string>

namespace Display {
class DisplayManager;
}

class DisplayConfigWidget : public QWidget {
  Q_OBJECT

 public:
  explicit DisplayConfigWidget(QWidget *parent = nullptr);
  ~DisplayConfigWidget();
  
  void SetDisplayManager(Display::DisplayManager *manager);
  void LoadConfig();
  void SaveConfig();

 private slots:
  void OnToggleDisplay(const std::string &display_name, bool visible);

 private:
  void InitUI();
  void UpdateDisplayVisibility(const std::string &display_name, bool visible);
  
  Display::DisplayManager *display_manager_{nullptr};
  QVBoxLayout *main_layout_{nullptr};
  QScrollArea *scroll_area_{nullptr};
  QWidget *scroll_content_{nullptr};
  std::map<std::string, QToolButton*> display_toggle_buttons_;
};

