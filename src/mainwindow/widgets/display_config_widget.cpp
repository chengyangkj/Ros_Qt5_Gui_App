#include "display_config_widget.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/virtual_display.h"
#include "config/config_manager.h"
#include "config/config_define.h"
#include "msg/msg_info.h"
#include "logger/logger.h"
#include <QDebug>
#include <QFrame>
#include <QMessageBox>
#include <QAbstractItemView>
#include <QInputDialog>
#include <QTimer>

DisplayConfigWidget::DisplayConfigWidget(QWidget *parent)
    : QWidget(parent), robot_color_(QColor(0, 0, 255)) {
  InitUI();
  QTimer::singleShot(3000, this, &DisplayConfigWidget::LoadConfig);
}

DisplayConfigWidget::~DisplayConfigWidget() {}

void DisplayConfigWidget::InitUI() {
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(10, 10, 10, 10);
  main_layout_->setSpacing(8);
  
  QLabel *title_label = new QLabel("配置管理", this);
  title_label->setStyleSheet(R"(
    QLabel {
      font-size: 16px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  main_layout_->addWidget(title_label);
  
  tab_widget_ = new QTabWidget(this);
  tab_widget_->setStyleSheet(R"(
    QTabWidget::pane {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      background-color: #ffffff;
    }
    QTabBar::tab {
      background-color: #f0f0f0;
      color: #333333;
      padding: 8px 16px;
      border-top-left-radius: 4px;
      border-top-right-radius: 4px;
      margin-right: 2px;
    }
    QTabBar::tab:selected {
      background-color: #ffffff;
      color: #1976d2;
      font-weight: bold;
    }
    QTabBar::tab:hover {
      background-color: #e0e0e0;
    }
  )");
  
  InitDisplayConfigTab();
  InitChannelConfigTab();
  InitKeyValueTab();
  InitImageConfigTab();
  InitRobotShapeTab();
  
  main_layout_->addWidget(tab_widget_);
}

void DisplayConfigWidget::InitDisplayConfigTab() {
  display_tab_ = new QWidget();
  QVBoxLayout *tab_layout = new QVBoxLayout(display_tab_);
  tab_layout->setContentsMargins(10, 10, 10, 10);
  tab_layout->setSpacing(8);
  
  QLabel *section_label = new QLabel("图层配置", display_tab_);
  section_label->setStyleSheet(R"(
    QLabel {
      font-size: 14px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  tab_layout->addWidget(section_label);
  
  display_scroll_area_ = new QScrollArea(display_tab_);
  display_scroll_area_->setWidgetResizable(true);
  display_scroll_area_->setFrameShape(QFrame::NoFrame);
  display_scroll_area_->setStyleSheet("QScrollArea { border: none; background-color: transparent; }");
  
  display_scroll_content_ = new QWidget();
  QVBoxLayout *scroll_layout = new QVBoxLayout(display_scroll_content_);
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
    QGroupBox *group_box = new QGroupBox(QString::fromStdString(display_name), display_scroll_content_);
    group_box->setStyleSheet(R"(
      QGroupBox {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        margin-top: 10px;
        padding-top: 10px;
        background-color: #f9f9f9;
      }
      QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 5px;
      }
    )");
    
    QVBoxLayout *group_layout = new QVBoxLayout(group_box);
    group_layout->setContentsMargins(10, 15, 10, 10);
    group_layout->setSpacing(8);
    
    QHBoxLayout *topic_layout = new QHBoxLayout();
    QLabel *topic_label = new QLabel("话题:", group_box);
    topic_label->setFixedWidth(60);
    QLineEdit *topic_edit = new QLineEdit(group_box);
    topic_edit->setPlaceholderText("输入话题名称");
    topic_edit->setStyleSheet(R"(
      QLineEdit {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        padding: 4px;
        background-color: #ffffff;
      }
      QLineEdit:focus {
        border-color: #1976d2;
      }
    )");
    display_topic_edits_[display_name] = topic_edit;
    connect(topic_edit, &QLineEdit::editingFinished, [this, display_name, topic_edit]() {
      OnDisplayTopicChanged(display_name, topic_edit->text());
    });
    topic_layout->addWidget(topic_label);
    topic_layout->addWidget(topic_edit);
    group_layout->addLayout(topic_layout);
    
    QHBoxLayout *control_layout = new QHBoxLayout();
    QLabel *visible_label = new QLabel("可见:", group_box);
    visible_label->setFixedWidth(60);
    QToolButton *toggle_btn = new QToolButton(group_box);
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
    
    control_layout->addWidget(visible_label);
    control_layout->addWidget(toggle_btn);
    control_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
    group_layout->addLayout(control_layout);
    
    scroll_layout->addWidget(group_box);
  }
  
  scroll_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  display_scroll_area_->setWidget(display_scroll_content_);
  tab_layout->addWidget(display_scroll_area_);
  
  tab_widget_->addTab(display_tab_, "图层配置");
}

void DisplayConfigWidget::InitKeyValueTab() {
  key_value_tab_ = new QWidget();
  QVBoxLayout *tab_layout = new QVBoxLayout(key_value_tab_);
  tab_layout->setContentsMargins(10, 10, 10, 10);
  tab_layout->setSpacing(10);
  
  QLabel *section_label = new QLabel("键值配置", key_value_tab_);
  section_label->setStyleSheet(R"(
    QLabel {
      font-size: 14px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  tab_layout->addWidget(section_label);
  
  key_value_scroll_area_ = new QScrollArea(key_value_tab_);
  key_value_scroll_area_->setWidgetResizable(true);
  key_value_scroll_area_->setFrameShape(QFrame::NoFrame);
  key_value_scroll_area_->setStyleSheet("QScrollArea { border: none; background-color: transparent; }");
  
  key_value_scroll_content_ = new QWidget();
  QVBoxLayout *scroll_layout = new QVBoxLayout(key_value_scroll_content_);
  scroll_layout->setContentsMargins(0, 0, 0, 0);
  scroll_layout->setSpacing(5);
  
  key_value_scroll_area_->setWidget(key_value_scroll_content_);
  tab_layout->addWidget(key_value_scroll_area_);
  
  QPushButton *add_btn = new QPushButton("添加配置项", key_value_tab_);
  add_btn->setFixedWidth(120);
  add_btn->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #4caf50;
      border-radius: 4px;
      padding: 6px;
      background-color: #4caf50;
      color: #ffffff;
    }
    QPushButton:hover {
      background-color: #45a049;
    }
  )");
  connect(add_btn, &QPushButton::clicked, this, &DisplayConfigWidget::OnAddKeyValue);
  
  tab_layout->addWidget(add_btn, 0, Qt::AlignLeft);
  
  tab_widget_->addTab(key_value_tab_, "键值配置");
  
  RefreshKeyValueTab();
}

void DisplayConfigWidget::RefreshKeyValueTab() {
  QLayoutItem* item;
  while ((item = key_value_scroll_content_->layout()->takeAt(0)) != nullptr) {
    if (item->widget()) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  key_value_edits_.clear();
  
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  
  for (const auto &[key, value] : config.key_value) {
    QWidget *item_widget = new QWidget(key_value_scroll_content_);
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
    
    QLabel *key_label = new QLabel(QString::fromStdString(key) + ":", item_widget);
    key_label->setFixedWidth(150);
    key_label->setStyleSheet(R"(
      QLabel {
        font-size: 12px;
        color: #333333;
        font-weight: bold;
      }
    )");
    item_layout->addWidget(key_label);
    
    QLineEdit *value_edit = new QLineEdit(QString::fromStdString(value), item_widget);
    value_edit->setPlaceholderText("输入值");
    value_edit->setStyleSheet(R"(
      QLineEdit {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        padding: 4px;
        background-color: #ffffff;
      }
      QLineEdit:focus {
        border-color: #1976d2;
      }
    )");
    key_value_edits_[key] = value_edit;
    connect(value_edit, &QLineEdit::editingFinished, [this, key, value_edit]() {
      OnKeyValueChanged(key, value_edit->text());
    });
    item_layout->addWidget(value_edit);
    
    QPushButton *remove_btn = new QPushButton("删除", item_widget);
    remove_btn->setFixedWidth(60);
    remove_btn->setStyleSheet(R"(
      QPushButton {
        border: 1px solid #f44336;
        border-radius: 4px;
        padding: 4px;
        background-color: #f44336;
        color: #ffffff;
      }
      QPushButton:hover {
        background-color: #da190b;
      }
    )");
    connect(remove_btn, &QPushButton::clicked, [this, key]() {
      OnRemoveKeyValue(key);
    });
    item_layout->addWidget(remove_btn);
    
    key_value_scroll_content_->layout()->addWidget(item_widget);
  }
  
  key_value_scroll_content_->layout()->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
}

void DisplayConfigWidget::InitImageConfigTab() {
  image_tab_ = new QWidget();
  QVBoxLayout *tab_layout = new QVBoxLayout(image_tab_);
  tab_layout->setContentsMargins(10, 10, 10, 10);
  tab_layout->setSpacing(10);
  
  QLabel *section_label = new QLabel("图像配置", image_tab_);
  section_label->setStyleSheet(R"(
    QLabel {
      font-size: 14px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  tab_layout->addWidget(section_label);
  
  image_table_ = new QTableWidget(0, 4, image_tab_);
  image_table_->setHorizontalHeaderLabels(QStringList() << "位置" << "话题" << "启用" << "操作");
  image_table_->horizontalHeader()->setStretchLastSection(true);
  image_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  image_table_->setStyleSheet(R"(
    QTableWidget {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      background-color: #ffffff;
      gridline-color: #e0e0e0;
    }
    QTableWidget::item {
      padding: 4px;
    }
    QHeaderView::section {
      background-color: #f0f0f0;
      padding: 4px;
      border: none;
      border-bottom: 1px solid #d0d0d0;
    }
  )");
  
  connect(image_table_, &QTableWidget::cellChanged, this, &DisplayConfigWidget::OnImageConfigChanged);
  
  QPushButton *add_btn = new QPushButton("添加图像", image_tab_);
  add_btn->setFixedWidth(100);
  add_btn->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #4caf50;
      border-radius: 4px;
      padding: 6px;
      background-color: #4caf50;
      color: #ffffff;
    }
    QPushButton:hover {
      background-color: #45a049;
    }
  )");
  connect(add_btn, &QPushButton::clicked, this, &DisplayConfigWidget::OnAddImageConfig);
  
  tab_layout->addWidget(image_table_);
  tab_layout->addWidget(add_btn, 0, Qt::AlignLeft);
  
  tab_widget_->addTab(image_tab_, "图像配置");
}

void DisplayConfigWidget::InitRobotShapeTab() {
  robot_shape_tab_ = new QWidget();
  QVBoxLayout *tab_layout = new QVBoxLayout(robot_shape_tab_);
  tab_layout->setContentsMargins(10, 10, 10, 10);
  tab_layout->setSpacing(10);
  
  QLabel *section_label = new QLabel("机器人外形配置", robot_shape_tab_);
  section_label->setStyleSheet(R"(
    QLabel {
      font-size: 14px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  tab_layout->addWidget(section_label);
  
  QGroupBox *points_group = new QGroupBox("外形点列表", robot_shape_tab_);
  points_group->setStyleSheet(R"(
    QGroupBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      margin-top: 10px;
      padding-top: 10px;
      background-color: #f9f9f9;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 5px;
    }
  )");
  
  QVBoxLayout *points_layout = new QVBoxLayout(points_group);
  points_layout->setContentsMargins(10, 15, 10, 10);
  points_layout->setSpacing(8);
  
  robot_points_table_ = new QTableWidget(0, 2, points_group);
  robot_points_table_->setHorizontalHeaderLabels(QStringList() << "X" << "Y");
  robot_points_table_->horizontalHeader()->setStretchLastSection(true);
  robot_points_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  robot_points_table_->setStyleSheet(R"(
    QTableWidget {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      background-color: #ffffff;
      gridline-color: #e0e0e0;
    }
    QTableWidget::item {
      padding: 4px;
    }
    QHeaderView::section {
      background-color: #f0f0f0;
      padding: 4px;
      border: none;
      border-bottom: 1px solid #d0d0d0;
    }
  )");
  
  connect(robot_points_table_, &QTableWidget::cellChanged, this, &DisplayConfigWidget::OnRobotShapePointChanged);
  
  QHBoxLayout *points_btn_layout = new QHBoxLayout();
  QPushButton *add_point_btn = new QPushButton("添加点", points_group);
  add_point_btn->setFixedWidth(80);
  add_point_btn->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #4caf50;
      border-radius: 4px;
      padding: 4px;
      background-color: #4caf50;
      color: #ffffff;
    }
    QPushButton:hover {
      background-color: #45a049;
    }
  )");
  connect(add_point_btn, &QPushButton::clicked, [this]() {
    int row = robot_points_table_->rowCount();
    robot_points_table_->insertRow(row);
    robot_points_table_->setItem(row, 0, new QTableWidgetItem("0.0"));
    robot_points_table_->setItem(row, 1, new QTableWidgetItem("0.0"));
    OnRobotShapePointChanged();
  });
  
  QPushButton *remove_point_btn = new QPushButton("删除点", points_group);
  remove_point_btn->setFixedWidth(80);
  remove_point_btn->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #f44336;
      border-radius: 4px;
      padding: 4px;
      background-color: #f44336;
      color: #ffffff;
    }
    QPushButton:hover {
      background-color: #da190b;
    }
  )");
  connect(remove_point_btn, &QPushButton::clicked, [this]() {
    int row = robot_points_table_->currentRow();
    if (row >= 0) {
      robot_points_table_->removeRow(row);
      OnRobotShapePointChanged();
    }
  });
  
  points_btn_layout->addWidget(add_point_btn);
  points_btn_layout->addWidget(remove_point_btn);
  points_btn_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  
  points_layout->addWidget(robot_points_table_);
  points_layout->addLayout(points_btn_layout);
  
  QGroupBox *style_group = new QGroupBox("样式设置", robot_shape_tab_);
  style_group->setStyleSheet(R"(
    QGroupBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      margin-top: 10px;
      padding-top: 10px;
      background-color: #f9f9f9;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 5px;
    }
  )");
  
  QVBoxLayout *style_layout = new QVBoxLayout(style_group);
  style_layout->setContentsMargins(10, 15, 10, 10);
  style_layout->setSpacing(10);
  
  robot_is_ellipse_checkbox_ = new QCheckBox("使用椭圆", style_group);
  robot_is_ellipse_checkbox_->setStyleSheet(R"(
    QCheckBox {
      spacing: 5px;
    }
    QCheckBox::indicator {
      width: 18px;
      height: 18px;
      border: 1px solid #d0d0d0;
      border-radius: 3px;
      background-color: #ffffff;
    }
    QCheckBox::indicator:checked {
      background-color: #4caf50;
      border-color: #4caf50;
    }
  )");
  connect(robot_is_ellipse_checkbox_, &QCheckBox::toggled, this, &DisplayConfigWidget::OnRobotShapeIsEllipseChanged);
  
  QHBoxLayout *color_layout = new QHBoxLayout();
  QLabel *color_label = new QLabel("颜色:", style_group);
  color_label->setFixedWidth(80);
  robot_color_button_ = new QPushButton("选择颜色", style_group);
  robot_color_button_->setFixedWidth(120);
  robot_color_button_->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px;
      background-color: #ffffff;
    }
    QPushButton:hover {
      background-color: #f0f0f0;
      border-color: #1976d2;
    }
  )");
  connect(robot_color_button_, &QPushButton::clicked, this, &DisplayConfigWidget::OnRobotShapeColorChanged);
  color_layout->addWidget(color_label);
  color_layout->addWidget(robot_color_button_);
  color_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  
  QHBoxLayout *opacity_layout = new QHBoxLayout();
  QLabel *opacity_label = new QLabel("透明度:", style_group);
  opacity_label->setFixedWidth(80);
  robot_opacity_slider_ = new QSlider(Qt::Horizontal, style_group);
  robot_opacity_slider_->setRange(0, 100);
  robot_opacity_slider_->setValue(50);
  robot_opacity_slider_->setStyleSheet(R"(
    QSlider::groove:horizontal {
      border: 1px solid #d0d0d0;
      height: 6px;
      background: #e0e0e0;
      border-radius: 3px;
    }
    QSlider::handle:horizontal {
      background: #1976d2;
      border: 1px solid #1976d2;
      width: 18px;
      margin: -2px 0;
      border-radius: 9px;
    }
    QSlider::handle:horizontal:hover {
      background: #1565c0;
    }
  )");
  robot_opacity_label_ = new QLabel("50%", style_group);
  robot_opacity_label_->setFixedWidth(50);
  connect(robot_opacity_slider_, &QSlider::valueChanged, [this](int value) {
    robot_opacity_label_->setText(QString::number(value) + "%");
    OnRobotShapeOpacityChanged(value);
  });
  opacity_layout->addWidget(opacity_label);
  opacity_layout->addWidget(robot_opacity_slider_);
  opacity_layout->addWidget(robot_opacity_label_);
  
  style_layout->addWidget(robot_is_ellipse_checkbox_);
  style_layout->addLayout(color_layout);
  style_layout->addLayout(opacity_layout);
  
  tab_layout->addWidget(points_group);
  tab_layout->addWidget(style_group);
  tab_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  
  tab_widget_->addTab(robot_shape_tab_, "机器人外形");
}

void DisplayConfigWidget::InitChannelConfigTab() {
  channel_config_tab_ = new QWidget();
  QVBoxLayout *tab_layout = new QVBoxLayout(channel_config_tab_);
  tab_layout->setContentsMargins(10, 10, 10, 10);
  tab_layout->setSpacing(10);
  
  QLabel *section_label = new QLabel("通信配置", channel_config_tab_);
  section_label->setStyleSheet(R"(
    QLabel {
      font-size: 14px;
      font-weight: bold;
      color: #333333;
      padding: 5px;
    }
  )");
  tab_layout->addWidget(section_label);
  
  QGroupBox *channel_type_group = new QGroupBox("通道类型", channel_config_tab_);
  channel_type_group->setStyleSheet(R"(
    QGroupBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      margin-top: 10px;
      padding-top: 10px;
      background-color: #f9f9f9;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 5px;
    }
  )");
  
  QVBoxLayout *channel_type_layout = new QVBoxLayout(channel_type_group);
  channel_type_layout->setContentsMargins(10, 15, 10, 10);
  channel_type_layout->setSpacing(8);
  
  QHBoxLayout *type_layout = new QHBoxLayout();
  QLabel *type_label = new QLabel("类型:", channel_type_group);
  type_label->setFixedWidth(80);
  channel_type_combo_ = new QComboBox(channel_type_group);
  channel_type_combo_->addItem("auto", "auto");
  channel_type_combo_->addItem("ROS2", "ros2");
  channel_type_combo_->addItem("ROS1", "ros1");
  channel_type_combo_->addItem("ROSBridge", "rosbridge");
  channel_type_combo_->setStyleSheet(R"(
    QComboBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px;
      background-color: #ffffff;
      min-width: 150px;
    }
    QComboBox:hover {
      border-color: #1976d2;
    }
    QComboBox::drop-down {
      border: none;
      width: 20px;
    }
    QComboBox::down-arrow {
      image: none;
      border-left: 4px solid transparent;
      border-right: 4px solid transparent;
      border-top: 6px solid #333333;
      margin-right: 5px;
    }
  )");
  connect(channel_type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
    // 如果正在加载配置，不显示提示
    if (is_loading_config_) {
      return;
    }
    
    auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
    QString channel_type = channel_type_combo_->itemData(index).toString();
    QString old_channel_type = QString::fromStdString(config.channel_config.channel_type);
    
    // 更新通道配置
    config.channel_config.channel_type = channel_type.toStdString();
    AutoSaveConfig();
    
    // 根据选择的类型显示/隐藏 ROSBridge 配置
    bool show_rosbridge = (channel_type == "rosbridge");
    rosbridge_ip_edit_->setEnabled(show_rosbridge);
    rosbridge_port_edit_->setEnabled(show_rosbridge);
    
    // 如果通道类型发生变化，提示用户需要重启
    if (channel_type != old_channel_type) {
      QMessageBox::information(this, "通道配置已更改", 
                                "通道类型已更改为: " + channel_type + "\n\n"
                                "请重启应用程序以使配置生效。",
                                QMessageBox::Ok);
    }
  });
  type_layout->addWidget(type_label);
  type_layout->addWidget(channel_type_combo_);
  type_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  channel_type_layout->addLayout(type_layout);
  
  tab_layout->addWidget(channel_type_group);
  
  QGroupBox *rosbridge_group = new QGroupBox("ROSBridge 配置", channel_config_tab_);
  rosbridge_group->setStyleSheet(R"(
    QGroupBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      margin-top: 10px;
      padding-top: 10px;
      background-color: #f9f9f9;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 5px;
    }
  )");
  
  QVBoxLayout *rosbridge_layout = new QVBoxLayout(rosbridge_group);
  rosbridge_layout->setContentsMargins(10, 15, 10, 10);
  rosbridge_layout->setSpacing(8);
  
  QHBoxLayout *ip_layout = new QHBoxLayout();
  QLabel *ip_label = new QLabel("IP地址:", rosbridge_group);
  ip_label->setFixedWidth(80);
  rosbridge_ip_edit_ = new QLineEdit(rosbridge_group);
  rosbridge_ip_edit_->setPlaceholderText("127.0.0.1");
  rosbridge_ip_edit_->setStyleSheet(R"(
    QLineEdit {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px;
      background-color: #ffffff;
    }
    QLineEdit:focus {
      border-color: #1976d2;
    }
    QLineEdit:disabled {
      background-color: #f5f5f5;
      color: #999999;
    }
  )");
  connect(rosbridge_ip_edit_, &QLineEdit::editingFinished, [this]() {
    // 如果正在加载配置，不显示提示
    if (is_loading_config_) {
      return;
    }
    
    auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
    QString new_ip = rosbridge_ip_edit_->text();
    QString old_ip = QString::fromStdString(config.channel_config.rosbridge_config.ip);
    
    // 更新 ROSBridge IP 配置
    config.channel_config.rosbridge_config.ip = new_ip.toStdString();
    AutoSaveConfig();
    
    // 如果 IP 地址发生变化，提示用户需要重启
    if (new_ip != old_ip && !new_ip.isEmpty()) {
      QMessageBox::information(this, "ROSBridge 配置已更改", 
                                "ROSBridge IP 地址已更改为: " + new_ip + "\n\n"
                                "请重启应用程序以使配置生效。",
                                QMessageBox::Ok);
    }
  });
  ip_layout->addWidget(ip_label);
  ip_layout->addWidget(rosbridge_ip_edit_);
  rosbridge_layout->addLayout(ip_layout);
  
  QHBoxLayout *port_layout = new QHBoxLayout();
  QLabel *port_label = new QLabel("端口:", rosbridge_group);
  port_label->setFixedWidth(80);
  rosbridge_port_edit_ = new QLineEdit(rosbridge_group);
  rosbridge_port_edit_->setPlaceholderText("9090");
  rosbridge_port_edit_->setStyleSheet(R"(
    QLineEdit {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px;
      background-color: #ffffff;
    }
    QLineEdit:focus {
      border-color: #1976d2;
    }
    QLineEdit:disabled {
      background-color: #f5f5f5;
      color: #999999;
    }
  )");
  connect(rosbridge_port_edit_, &QLineEdit::editingFinished, [this]() {
    // 如果正在加载配置，不显示提示
    if (is_loading_config_) {
      return;
    }
    
    auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
    QString new_port = rosbridge_port_edit_->text();
    QString old_port = QString::fromStdString(config.channel_config.rosbridge_config.port);
    
    // 更新 ROSBridge 端口配置
    config.channel_config.rosbridge_config.port = new_port.toStdString();
    AutoSaveConfig();
    
    // 如果端口发生变化，提示用户需要重启
    if (new_port != old_port && !new_port.isEmpty()) {
      QMessageBox::information(this, "ROSBridge 配置已更改", 
                                "ROSBridge 端口已更改为: " + new_port + "\n\n"
                                "请重启应用程序以使配置生效。",
                                QMessageBox::Ok);
    }
  });
  port_layout->addWidget(port_label);
  port_layout->addWidget(rosbridge_port_edit_);
  rosbridge_layout->addLayout(port_layout);
  
  tab_layout->addWidget(rosbridge_group);
  
  reconnect_channel_btn_ = new QPushButton("保存", channel_config_tab_);
  reconnect_channel_btn_->setFixedWidth(150);
  reconnect_channel_btn_->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #1976d2;
      border-radius: 4px;
      padding: 6px;
      background-color: #1976d2;
      color: #ffffff;
      font-weight: bold;
    }
    QPushButton:hover {
      background-color: #1565c0;
    }
    QPushButton:pressed {
      background-color: #0d47a1;
    }
  )");
  connect(reconnect_channel_btn_, &QPushButton::clicked, [this]() {
    QMessageBox::information(this, "提示", 
                              "请重启应用程序以使通道配置生效。\n"
                              "当前配置已保存。",
                              QMessageBox::Ok);
  });
  tab_layout->addWidget(reconnect_channel_btn_, 0, Qt::AlignLeft);
  
  tab_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  
  tab_widget_->addTab(channel_config_tab_, "通信配置");
}

void DisplayConfigWidget::SetDisplayManager(Display::DisplayManager *manager) {
  display_manager_ = manager;
  if (display_manager_) {
  LoadConfig();
  }
}

void DisplayConfigWidget::OnToggleDisplay(const std::string &display_name, bool visible) {
  UpdateDisplayVisibility(display_name, visible);
  AutoSaveConfig();
}

void DisplayConfigWidget::OnDisplayTopicChanged(const std::string &display_name, const QString &topic) {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  auto it = std::find_if(config.display_config.begin(), config.display_config.end(),
                        [&display_name](const auto &item) {
                          return item.display_name == display_name;
                        });
  if (it != config.display_config.end()) {
    it->topic = topic.toStdString();
  } else {
    config.display_config.push_back(Config::DisplayConfig(display_name, topic.toStdString(), true));
  }
  AutoSaveConfig();
}

void DisplayConfigWidget::OnKeyValueChanged(const std::string &key, const QString &value) {
  SET_KEY_VALUE(key, value.toStdString())
}

void DisplayConfigWidget::OnAddKeyValue() {
  bool ok;
  QString key = QInputDialog::getText(this, "添加配置项", "请输入键名:", QLineEdit::Normal, "", &ok);
  if (ok && !key.isEmpty()) {
    QString value = QInputDialog::getText(this, "添加配置项", "请输入值:", QLineEdit::Normal, "", &ok);
    if (ok) {
      SET_KEY_VALUE(key.toStdString(), value.toStdString())
      RefreshKeyValueTab();
    }
  }
}

void DisplayConfigWidget::OnRemoveKeyValue(const std::string &key) {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  config.key_value.erase(key);
  AutoSaveConfig();
  RefreshKeyValueTab();
}

void DisplayConfigWidget::OnAddImageConfig() {
  int row = image_table_->rowCount();
  image_table_->insertRow(row);
  
  QTableWidgetItem *location_item = new QTableWidgetItem("");
  QTableWidgetItem *topic_item = new QTableWidgetItem("");
  QTableWidgetItem *enable_item = new QTableWidgetItem("true");
  enable_item->setFlags(enable_item->flags() & ~Qt::ItemIsEditable);
  
  QCheckBox *enable_checkbox = new QCheckBox();
  enable_checkbox->setChecked(true);
  enable_checkbox->setStyleSheet(R"(
    QCheckBox {
      spacing: 5px;
    }
    QCheckBox::indicator {
      width: 18px;
      height: 18px;
      border: 1px solid #d0d0d0;
      border-radius: 3px;
      background-color: #ffffff;
    }
    QCheckBox::indicator:checked {
      background-color: #4caf50;
      border-color: #4caf50;
    }
  )");
  connect(enable_checkbox, &QCheckBox::toggled, [this, row](bool checked) {
    image_table_->item(row, 2)->setText(checked ? "true" : "false");
    OnImageConfigChanged(row);
  });
  
  QPushButton *remove_btn = new QPushButton("删除");
  remove_btn->setStyleSheet(R"(
    QPushButton {
      border: 1px solid #f44336;
      border-radius: 4px;
      padding: 2px 8px;
      background-color: #f44336;
      color: #ffffff;
    }
    QPushButton:hover {
      background-color: #da190b;
    }
  )");
  connect(remove_btn, &QPushButton::clicked, [this, row]() {
    OnRemoveImageConfig(row);
  });
  
  image_table_->setItem(row, 0, location_item);
  image_table_->setItem(row, 1, topic_item);
  image_table_->setItem(row, 2, enable_item);
  image_table_->setCellWidget(row, 2, enable_checkbox);
  image_table_->setCellWidget(row, 3, remove_btn);
  
  OnImageConfigChanged(row);
}

void DisplayConfigWidget::OnRemoveImageConfig(int row) {
  image_table_->removeRow(row);
  
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  if (row < static_cast<int>(config.images.size())) {
    config.images.erase(config.images.begin() + row);
    AutoSaveConfig();
  }
  
  for (int i = row; i < image_table_->rowCount(); i++) {
    QPushButton *btn = qobject_cast<QPushButton*>(image_table_->cellWidget(i, 3));
    if (btn) {
      btn->disconnect();
      connect(btn, &QPushButton::clicked, [this, i]() {
        OnRemoveImageConfig(i);
      });
    }
    QCheckBox *checkbox = qobject_cast<QCheckBox*>(image_table_->cellWidget(i, 2));
    if (checkbox) {
      checkbox->disconnect();
      connect(checkbox, &QCheckBox::toggled, [this, i](bool checked) {
        image_table_->item(i, 2)->setText(checked ? "true" : "false");
        OnImageConfigChanged(i);
      });
    }
  }
}

void DisplayConfigWidget::OnImageConfigChanged(int row) {
  if (row < 0 || row >= image_table_->rowCount()) {
    return;
  }
  
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  
  QTableWidgetItem *location_item = image_table_->item(row, 0);
  QTableWidgetItem *topic_item = image_table_->item(row, 1);
  QCheckBox *enable_checkbox = qobject_cast<QCheckBox*>(image_table_->cellWidget(row, 2));
  
  if (!location_item || !topic_item || !enable_checkbox) {
    return;
  }
  
  Config::ImageDisplayConfig image_config;
  image_config.location = location_item->text().toStdString();
  image_config.topic = topic_item->text().toStdString();
  image_config.enable = enable_checkbox->isChecked();
  
  if (row < static_cast<int>(config.images.size())) {
    config.images[row] = image_config;
  } else {
    config.images.push_back(image_config);
  }
  
  AutoSaveConfig();
}

void DisplayConfigWidget::OnRobotShapePointChanged() {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  config.robot_shape_config.shaped_points.clear();
  
  for (int row = 0; row < robot_points_table_->rowCount(); row++) {
    QTableWidgetItem *x_item = robot_points_table_->item(row, 0);
    QTableWidgetItem *y_item = robot_points_table_->item(row, 1);
    
    if (x_item && y_item) {
      bool x_ok, y_ok;
      double x = x_item->text().toDouble(&x_ok);
      double y = y_item->text().toDouble(&y_ok);
      
      if (x_ok && y_ok) {
        config.robot_shape_config.shaped_points.push_back({x, y});
      }
    }
  }
  
  AutoSaveConfig();
}

void DisplayConfigWidget::OnRobotShapeIsEllipseChanged(bool checked) {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  config.robot_shape_config.is_ellipse = checked;
  AutoSaveConfig();
}

void DisplayConfigWidget::OnRobotShapeColorChanged() {
  QColor color = QColorDialog::getColor(robot_color_, this, "选择颜色");
  if (color.isValid()) {
    robot_color_ = color;
    QString color_style = QString("background-color: %1;").arg(color.name());
    robot_color_button_->setStyleSheet(R"(
      QPushButton {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        padding: 4px;
      }
      QPushButton:hover {
        background-color: #f0f0f0;
        border-color: #1976d2;
      }
    )" + color_style);
    
    auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
    QString color_str = QString("0x%1").arg(color.rgb(), 8, 16, QChar('0')).toUpper();
    config.robot_shape_config.color = color_str.toStdString();
    AutoSaveConfig();
  }
}

void DisplayConfigWidget::OnRobotShapeOpacityChanged(int value) {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  config.robot_shape_config.opacity = value / 100.0f;
  AutoSaveConfig();
}

void DisplayConfigWidget::UpdateDisplayVisibility(const std::string &display_name, bool visible) {
  auto display = Display::FactoryDisplay::Instance()->GetDisplay(display_name);
  if (display) {
    display->setVisible(visible);
    LOG_INFO("Display " << display_name << " visibility set to " << (visible ? "visible" : "hidden"));
  }
}

void DisplayConfigWidget::AutoSaveConfig() {
  Config::ConfigManager::Instacnce()->StoreConfig();
}

void DisplayConfigWidget::LoadConfig() {
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  
  for (auto &display_config : config.display_config) {
    auto toggle_it = display_toggle_buttons_.find(display_config.display_name);
    if (toggle_it != display_toggle_buttons_.end()) {
      toggle_it->second->blockSignals(true);
      toggle_it->second->setChecked(display_config.visible);
      toggle_it->second->setText(display_config.visible ? "✓" : "");
      toggle_it->second->blockSignals(false);
      UpdateDisplayVisibility(display_config.display_name, display_config.visible);
    }
    
    auto topic_it = display_topic_edits_.find(display_config.display_name);
    if (topic_it != display_topic_edits_.end()) {
      topic_it->second->blockSignals(true);
      topic_it->second->setText(QString::fromStdString(display_config.topic));
      topic_it->second->blockSignals(false);
    }
  }
  
  RefreshKeyValueTab();
  
  image_table_->blockSignals(true);
  image_table_->setRowCount(0);
  for (size_t i = 0; i < config.images.size(); i++) {
    const auto &image_config = config.images[i];
    int row = image_table_->rowCount();
    image_table_->insertRow(row);
    
    QTableWidgetItem *location_item = new QTableWidgetItem(QString::fromStdString(image_config.location));
    QTableWidgetItem *topic_item = new QTableWidgetItem(QString::fromStdString(image_config.topic));
    QTableWidgetItem *enable_item = new QTableWidgetItem(image_config.enable ? "true" : "false");
    enable_item->setFlags(enable_item->flags() & ~Qt::ItemIsEditable);
    
    QCheckBox *enable_checkbox = new QCheckBox();
    enable_checkbox->setChecked(image_config.enable);
    enable_checkbox->setStyleSheet(R"(
      QCheckBox {
        spacing: 5px;
      }
      QCheckBox::indicator {
        width: 18px;
        height: 18px;
        border: 1px solid #d0d0d0;
        border-radius: 3px;
        background-color: #ffffff;
      }
      QCheckBox::indicator:checked {
        background-color: #4caf50;
        border-color: #4caf50;
      }
    )");
    connect(enable_checkbox, &QCheckBox::toggled, [this, row](bool checked) {
      image_table_->item(row, 2)->setText(checked ? "true" : "false");
      OnImageConfigChanged(row);
    });
    
    QPushButton *remove_btn = new QPushButton("删除");
    remove_btn->setStyleSheet(R"(
      QPushButton {
        border: 1px solid #f44336;
        border-radius: 4px;
        padding: 2px 8px;
        background-color: #f44336;
        color: #ffffff;
      }
      QPushButton:hover {
        background-color: #da190b;
      }
    )");
    connect(remove_btn, &QPushButton::clicked, [this, row]() {
      OnRemoveImageConfig(row);
    });
    
    image_table_->setItem(row, 0, location_item);
    image_table_->setItem(row, 1, topic_item);
    image_table_->setItem(row, 2, enable_item);
    image_table_->setCellWidget(row, 2, enable_checkbox);
    image_table_->setCellWidget(row, 3, remove_btn);
  }
  image_table_->blockSignals(false);
  
  robot_points_table_->blockSignals(true);
  robot_points_table_->setRowCount(0);
  for (const auto &point : config.robot_shape_config.shaped_points) {
    int row = robot_points_table_->rowCount();
    robot_points_table_->insertRow(row);
    robot_points_table_->setItem(row, 0, new QTableWidgetItem(QString::number(point.x)));
    robot_points_table_->setItem(row, 1, new QTableWidgetItem(QString::number(point.y)));
  }
  robot_points_table_->blockSignals(false);
  
  robot_is_ellipse_checkbox_->blockSignals(true);
  robot_is_ellipse_checkbox_->setChecked(config.robot_shape_config.is_ellipse);
  robot_is_ellipse_checkbox_->blockSignals(false);
  
  QString color_str = QString::fromStdString(config.robot_shape_config.color);
  if (color_str.startsWith("0x")) {
    bool ok;
    uint rgb = color_str.mid(2).toUInt(&ok, 16);
    if (ok) {
      robot_color_ = QColor::fromRgb(rgb);
      QString color_style = QString("background-color: %1;").arg(robot_color_.name());
      robot_color_button_->setStyleSheet(R"(
        QPushButton {
          border: 1px solid #d0d0d0;
          border-radius: 4px;
          padding: 4px;
        }
        QPushButton:hover {
          background-color: #f0f0f0;
          border-color: #1976d2;
        }
      )" + color_style);
    }
  }
  
  robot_opacity_slider_->blockSignals(true);
  robot_opacity_slider_->setValue(static_cast<int>(config.robot_shape_config.opacity * 100));
  robot_opacity_label_->setText(QString::number(robot_opacity_slider_->value()) + "%");
  robot_opacity_slider_->blockSignals(false);
  
  // Load channel config
  is_loading_config_ = true;
  
  // 加载通道类型
  std::string channel_type = config.channel_config.channel_type.empty() ? "auto" : config.channel_config.channel_type;
  channel_type_combo_->blockSignals(true);
  int index = channel_type_combo_->findData(QString::fromStdString(channel_type));
  if (index >= 0) {
    channel_type_combo_->setCurrentIndex(index);
  } else {
    channel_type_combo_->setCurrentIndex(0); // default to "auto"
  }
  channel_type_combo_->blockSignals(false);
  
  // 加载 ROSBridge IP
  std::string rosbridge_ip = config.channel_config.rosbridge_config.ip.empty() ? "127.0.0.1" : config.channel_config.rosbridge_config.ip;
  rosbridge_ip_edit_->blockSignals(true);
  rosbridge_ip_edit_->setText(QString::fromStdString(rosbridge_ip));
  rosbridge_ip_edit_->blockSignals(false);
  
  // 加载 ROSBridge 端口
  std::string rosbridge_port = config.channel_config.rosbridge_config.port.empty() ? "9090" : config.channel_config.rosbridge_config.port;
  rosbridge_port_edit_->blockSignals(true);
  rosbridge_port_edit_->setText(QString::fromStdString(rosbridge_port));
  rosbridge_port_edit_->blockSignals(false);
  
  // Enable/disable ROSBridge config based on channel type
  bool show_rosbridge = (channel_type == "rosbridge");
  rosbridge_ip_edit_->setEnabled(show_rosbridge);
  rosbridge_port_edit_->setEnabled(show_rosbridge);
  
  is_loading_config_ = false;
}

void DisplayConfigWidget::SaveConfig() {
  AutoSaveConfig();
}

