#include "display_config_widget.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/virtual_display.h"
#include "config/config_manager.h"
#include "logger/logger.h"
#include <QDebug>
#include <QFrame>

DisplayConfigWidget::DisplayConfigWidget(QWidget *parent)
    : QWidget(parent) {
  InitUI();
}

DisplayConfigWidget::~DisplayConfigWidget() {}

void DisplayConfigWidget::InitUI() {
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(10, 10, 10, 10);
  main_layout_->setSpacing(8);
  
  QLabel *title_label = new QLabel("图层配置", this);
  title_label->setStyleSheet(R"(
    QLabel {
      font-size: 16px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  main_layout_->addWidget(title_label);
  
  scroll_area_ = new QScrollArea(this);
  scroll_area_->setWidgetResizable(true);
  scroll_area_->setFrameShape(QFrame::NoFrame);
  scroll_area_->setStyleSheet("QScrollArea { border: none; background-color: transparent; }");
  
  scroll_content_ = new QWidget();
  QVBoxLayout *scroll_layout = new QVBoxLayout(scroll_content_);
  scroll_layout->setContentsMargins(0, 0, 0, 0);
  scroll_layout->setSpacing(5);
  
  std::vector<std::pair<std::string, std::string>> display_types = {
    {DISPLAY_MAP, ":/images/classes/Map.png"},
    {DISPLAY_ROBOT, ":/images/classes/RobotModel.png"},
    {DISPLAY_LASER, ":/images/classes/LaserScan.png"},
    {DISPLAY_GLOBAL_PATH, ":/images/classes/Path.png"},
    {DISPLAY_LOCAL_PATH, ":/images/classes/Path.png"},
    {DISPLAY_GLOBAL_COST_MAP, ":/images/classes/Grid.png"},
    {DISPLAY_LOCAL_COST_MAP, ":/images/classes/Grid.png"},
    {DISPLAY_ROBOT_FOOTPRINT, ":/images/classes/RobotLink.png"},
    {DISPLAY_GOAL, ":/images/classes/SetGoal.png"},
  };
  
  for (const auto &[display_name, icon_path] : display_types) {
    QWidget *item_widget = new QWidget(scroll_content_);
    item_widget->setStyleSheet(R"(
      QWidget {
        background-color: #f9f9f9;
        border-radius: 4px;
        padding: 5px;
      }
      QWidget:hover {
        background-color: #f0f0f0;
      }
    )");
    
    QHBoxLayout *item_layout = new QHBoxLayout(item_widget);
    item_layout->setContentsMargins(8, 5, 8, 5);
    item_layout->setSpacing(10);
    
    QToolButton *icon_btn = new QToolButton(item_widget);
    QIcon icon(QString::fromStdString(icon_path));
    if (icon.isNull()) {
      icon = QIcon(":/images/display.png");
    }
    icon_btn->setIcon(icon);
    icon_btn->setIconSize(QSize(24, 24));
    icon_btn->setEnabled(false);
    icon_btn->setStyleSheet(R"(
      QToolButton {
        border: none;
        background-color: transparent;
      }
    )");
    item_layout->addWidget(icon_btn);
    
    QLabel *name_label = new QLabel(QString::fromStdString(display_name), item_widget);
    name_label->setStyleSheet(R"(
      QLabel {
        font-size: 12px;
        color: #333333;
      }
    )");
    item_layout->addWidget(name_label);
    
    item_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
    
    QToolButton *toggle_btn = new QToolButton(item_widget);
    toggle_btn->setCheckable(true);
    toggle_btn->setChecked(true);
    toggle_btn->setText("✓");
    toggle_btn->setFixedSize(30, 24);
    toggle_btn->setStyleSheet(R"(
      QToolButton {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        background-color: #ffffff;
        color: #333333;
        font-weight: bold;
      }
      QToolButton:checked {
        background-color: #4caf50;
        border-color: #4caf50;
        color: #ffffff;
      }
      QToolButton:hover {
        border-color: #1976d2;
      }
    )");
    
    display_toggle_buttons_[display_name] = toggle_btn;
    
    connect(toggle_btn, &QToolButton::toggled, [this, display_name, toggle_btn](bool checked) {
      toggle_btn->setText(checked ? "✓" : "");
      OnToggleDisplay(display_name, checked);
    });
    
    item_layout->addWidget(toggle_btn);
    scroll_layout->addWidget(item_widget);
  }
  
  scroll_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  scroll_area_->setWidget(scroll_content_);
  main_layout_->addWidget(scroll_area_);
}

void DisplayConfigWidget::SetDisplayManager(Display::DisplayManager *manager) {
  display_manager_ = manager;
  LoadConfig();
}

void DisplayConfigWidget::OnToggleDisplay(const std::string &display_name, bool visible) {
  UpdateDisplayVisibility(display_name, visible);
  SaveConfig();
}

void DisplayConfigWidget::UpdateDisplayVisibility(const std::string &display_name, bool visible) {
  auto display = Display::FactoryDisplay::Instance()->GetDisplay(display_name);
  if (display) {
    display->setVisible(visible);
    LOG_INFO("Display " << display_name << " visibility set to " << (visible ? "visible" : "hidden"));
  }
}

void DisplayConfigWidget::LoadConfig() {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  for (auto &display_config : config.display_config) {
    auto it = display_toggle_buttons_.find(display_config.display_name);
    if (it != display_toggle_buttons_.end()) {
      it->second->blockSignals(true);
      it->second->setChecked(display_config.visible);
      it->second->setText(display_config.visible ? "✓" : "");
      it->second->blockSignals(false);
      
      UpdateDisplayVisibility(display_config.display_name, display_config.visible);
    }
  }
}

void DisplayConfigWidget::SaveConfig() {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  
  for (const auto &[display_name, toggle_btn] : display_toggle_buttons_) {
    bool visible = toggle_btn->isChecked();
    
    auto it = std::find_if(config.display_config.begin(), config.display_config.end(),
                          [&display_name](const auto &item) {
                            return item.display_name == display_name;
                          });
    
    if (it != config.display_config.end()) {
      it->visible = visible;
    } else {
      config.display_config.push_back(Config::DisplayConfig(display_name, "", true, visible));
    }
  }
  
  Config::ConfigManager::Instacnce()->StoreConfig();
}

