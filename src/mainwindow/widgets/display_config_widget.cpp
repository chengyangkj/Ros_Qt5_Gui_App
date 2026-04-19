#include "display_config_widget.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/virtual_display.h"
#include "config/config_manager.h"
#include "msg/msg_info.h"
#include "logger/logger.h"
#include <QAbstractItemView>
#include <QFileDialog>
#include <QFrame>
#include <QInputDialog>
#include <QMessageBox>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpacerItem>
#include <algorithm>

namespace {

QString ToggleStyle() {
  return QStringLiteral(
      "QToolButton { min-width:40px; max-width:40px; min-height:22px; max-height:22px; "
      "border-radius:11px; background:#E8EAED; border:none; }"
      "QToolButton:checked { background:#1a73e8; }"
      "QToolButton:hover { background:#DADCE0; }"
      "QToolButton:checked:hover { background:#1557b0; }");
}

QString LineEditStyle() {
  return QStringLiteral(
      "QLineEdit { border:1px solid rgba(0,0,0,0.12); border-radius:8px; padding:8px 10px; "
      "background:#fafafa; font-size:13px; }"
      "QLineEdit:focus { border-color:#1a73e8; background:#fff; }");
}

}  // namespace

DisplayConfigWidget::DisplayConfigWidget(QWidget *parent)
    : QWidget(parent), robot_color_(QColor(0, 0, 255)) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumWidth(420);
  ApplyGlobalStyle();
  InitUI();
}

DisplayConfigWidget::~DisplayConfigWidget() {}

void DisplayConfigWidget::ApplyGlobalStyle() {
  setStyleSheet(R"(
    DisplayConfigWidget {
      background-color: #eef1f6;
    }
    DisplayConfigWidget QLabel#pageTitle {
      font-size: 20px;
      font-weight: 600;
      color: #202124;
      padding-bottom: 4px;
    }
    DisplayConfigWidget QLabel#pageSubtitle {
      font-size: 13px;
      color: #5f6368;
      padding-bottom: 12px;
    }
    DisplayConfigWidget QListWidget#settingsNav {
      background-color: #e8eaed;
      border: none;
      border-radius: 12px;
      padding: 8px 6px;
      outline: none;
    }
    DisplayConfigWidget QListWidget#settingsNav::item {
      color: #3c4043;
      padding: 12px 14px;
      border-radius: 8px;
      margin: 2px 0;
      border: none;
    }
    DisplayConfigWidget QListWidget#settingsNav::item:hover {
      background-color: rgba(255,255,255,0.7);
    }
    DisplayConfigWidget QListWidget#settingsNav::item:selected {
      background-color: #ffffff;
      color: #1a73e8;
      font-weight: 600;
    }
    DisplayConfigWidget QScrollArea {
      border: none;
      background: transparent;
    }
    DisplayConfigWidget QToolTip {
      background: #fff;
      color: #202124;
      border: 1px solid rgba(0,0,0,0.12);
      padding: 6px 8px;
      border-radius: 4px;
    }
  )");
}

QFrame *DisplayConfigWidget::CreateSettingsCard(QWidget *parent) {
  QFrame *card = new QFrame(parent);
  card->setObjectName(QStringLiteral("settingsCard"));
  card->setStyleSheet(R"(
    QFrame#settingsCard {
      background-color: #ffffff;
      border: 1px solid rgba(0, 0, 0, 0.06);
      border-radius: 12px;
    }
  )");
  return card;
}

void DisplayConfigWidget::AddSectionHeader(QVBoxLayout *layout, const QString &title) {
  QLabel *lbl = new QLabel(title);
  lbl->setStyleSheet(
      QStringLiteral("QLabel { color: #5f6368; font-size: 12px; font-weight: 600; "
                     "letter-spacing: 0.3px; padding: 16px 4px 6px 4px; }"));
  layout->addWidget(lbl);
}

void DisplayConfigWidget::AddHintLabel(QVBoxLayout *layout, const QString &text) {
  QLabel *lbl = new QLabel(text);
  lbl->setObjectName(QStringLiteral("pageSubtitle"));
  lbl->setWordWrap(true);
  lbl->setStyleSheet(
      QStringLiteral("QLabel { font-size: 12px; color: #80868b; line-height: 1.45; padding: 0 2px 14px 2px; }"));
  layout->addWidget(lbl);
}

void DisplayConfigWidget::InitUI() {
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(16, 14, 16, 14);
  main_layout_->setSpacing(0);
  setAutoFillBackground(true);

  QLabel *title_label = new QLabel(QStringLiteral("Settings"), this);
  title_label->setStyleSheet(
      QStringLiteral("QLabel { font-size: 22px; font-weight: 700; color: #202124; padding: 4px 2px 14px 2px; }"));
  main_layout_->addWidget(title_label);

  QHBoxLayout *body = new QHBoxLayout();
  body->setSpacing(16);
  body->setContentsMargins(0, 0, 0, 0);

  nav_list_ = new QListWidget(this);
  nav_list_->setObjectName(QStringLiteral("settingsNav"));
  nav_list_->setMinimumWidth(200);
  nav_list_->setMaximumWidth(260);
  nav_list_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  nav_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nav_list_->setFocusPolicy(Qt::StrongFocus);
  const QStringList navTitles = {QStringLiteral("Channel"),
                                 QStringLiteral("Display & topics"),
                                 QStringLiteral("Cameras"),
                                 QStringLiteral("Robot shape"),
                                 QStringLiteral("Default map"),
                                 QStringLiteral("Key-value")};
  for (const QString &t : navTitles) {
    nav_list_->addItem(t);
  }

  page_stack_ = new QStackedWidget(this);
  page_stack_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  page_stack_->addWidget(CreateChannelPage());
  page_stack_->addWidget(CreateLayersPage());
  page_stack_->addWidget(CreateImagePage());
  page_stack_->addWidget(CreateRobotPage());
  page_stack_->addWidget(CreateMapPage());
  page_stack_->addWidget(CreateKeyValuePage());

  connect(nav_list_, &QListWidget::currentRowChanged, page_stack_, &QStackedWidget::setCurrentIndex);
  nav_list_->setCurrentRow(0);

  body->addWidget(nav_list_, 0);
  body->addWidget(page_stack_, 1);
  main_layout_->addLayout(body, 1);
}

QWidget *DisplayConfigWidget::CreateChannelPage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Channel"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to channel_config in config.json: channel_type (e.g. auto, rosbridge, ros1, ros2) "
      "and rosbridge_config.ip / port when using rosbridge."));

  AddSectionHeader(root, QStringLiteral("Connection"));

  QFrame *card = CreateSettingsCard(page);
  QVBoxLayout *card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(16, 14, 16, 16);
  card_layout->setSpacing(14);

  QHBoxLayout *type_layout = new QHBoxLayout();
  QLabel *type_label = new QLabel(QStringLiteral("Channel type"));
  type_label->setFixedWidth(88);
  type_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  channel_type_combo_ = new QComboBox(card);
  channel_type_combo_->setMinimumHeight(36);
  channel_type_combo_->setStyleSheet(
      QStringLiteral("QComboBox { border:1px solid #dadce0; border-radius:8px; padding:6px 12px; "
                     "background:#fff; min-height:22px; font-size:13px; }"
                     "QComboBox:hover { border-color:#1a73e8; }"
                     "QComboBox::drop-down { border:none; width:28px; }"));
  connect(channel_type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
    if (is_loading_config_) {
      return;
    }
    auto &config = Config::ConfigManager::Instance()->GetRootConfig();
    std::string channel_type = channel_type_combo_->itemData(index).toString().toStdString();
    QString old_channel_type = QString::fromStdString(config.channel_config.channel_type);
    config.channel_config.channel_type = channel_type;
    AutoSaveConfig();
    bool show_rosbridge = (channel_type == "rosbridge");
    rosbridge_ip_edit_->setEnabled(show_rosbridge);
    rosbridge_port_edit_->setEnabled(show_rosbridge);
    if (QString::fromStdString(channel_type) != old_channel_type) {
      QMessageBox::information(this, QStringLiteral("Channel"),
                               QStringLiteral("Channel type changed. Restart the application to apply."), QMessageBox::Ok);
    }
  });
  type_layout->addWidget(type_label);
  type_layout->addWidget(channel_type_combo_, 1);
  card_layout->addLayout(type_layout);

  QLabel *rb_title = new QLabel(QStringLiteral("ROSBridge"));
  rb_title->setStyleSheet(QStringLiteral("color:#80868b;font-size:11px;font-weight:600;padding-top:4px;"));
  card_layout->addWidget(rb_title);

  QHBoxLayout *ip_layout = new QHBoxLayout();
  QLabel *ip_label = new QLabel(QStringLiteral("Address"));
  ip_label->setFixedWidth(88);
  ip_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  rosbridge_ip_edit_ = new QLineEdit(card);
  rosbridge_ip_edit_->setPlaceholderText(QStringLiteral("127.0.0.1"));
  rosbridge_ip_edit_->setStyleSheet(LineEditStyle());
  connect(rosbridge_ip_edit_, &QLineEdit::editingFinished, [this]() {
    if (is_loading_config_) {
      return;
    }
    auto &config = Config::ConfigManager::Instance()->GetRootConfig();
    QString new_ip = rosbridge_ip_edit_->text();
    QString old_ip = QString::fromStdString(config.channel_config.rosbridge_config.ip);
    config.channel_config.rosbridge_config.ip = new_ip.toStdString();
    AutoSaveConfig();
    if (new_ip != old_ip && !new_ip.isEmpty()) {
      QMessageBox::information(this, QStringLiteral("ROSBridge"),
                               QStringLiteral("IP changed. Restart the application to apply."), QMessageBox::Ok);
    }
  });
  ip_layout->addWidget(ip_label);
  ip_layout->addWidget(rosbridge_ip_edit_, 1);
  card_layout->addLayout(ip_layout);

  QHBoxLayout *port_layout = new QHBoxLayout();
  QLabel *port_label = new QLabel(QStringLiteral("Port"));
  port_label->setFixedWidth(88);
  port_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  rosbridge_port_edit_ = new QLineEdit(card);
  rosbridge_port_edit_->setPlaceholderText(QStringLiteral("9090"));
  rosbridge_port_edit_->setStyleSheet(LineEditStyle());
  connect(rosbridge_port_edit_, &QLineEdit::editingFinished, [this]() {
    if (is_loading_config_) {
      return;
    }
    auto &config = Config::ConfigManager::Instance()->GetRootConfig();
    QString new_port = rosbridge_port_edit_->text();
    QString old_port = QString::fromStdString(config.channel_config.rosbridge_config.port);
    config.channel_config.rosbridge_config.port = new_port.toStdString();
    AutoSaveConfig();
    if (new_port != old_port && !new_port.isEmpty()) {
      QMessageBox::information(this, QStringLiteral("ROSBridge"),
                               QStringLiteral("Port changed. Restart the application to apply."), QMessageBox::Ok);
    }
  });
  port_layout->addWidget(port_label);
  port_layout->addWidget(rosbridge_port_edit_, 1);
  card_layout->addLayout(port_layout);

  root->addWidget(card);

  reconnect_channel_btn_ = new QPushButton(QStringLiteral("Help"), page);
  reconnect_channel_btn_->setCursor(Qt::PointingHandCursor);
  reconnect_channel_btn_->setStyleSheet(
      QStringLiteral("QPushButton { border:none; color:#1a73e8; font-size:13px; padding:10px 4px; background:transparent; }"
                     "QPushButton:hover { text-decoration:underline; color:#1557b0; }"));
  connect(reconnect_channel_btn_, &QPushButton::clicked, [this]() {
    QMessageBox::information(
        this, QStringLiteral("Channel"),
        QStringLiteral("Changes are saved to config.json.\nRestart the application after changing channel type or ROSBridge address."),
        QMessageBox::Ok);
  });
  root->addWidget(reconnect_channel_btn_, 0, Qt::AlignLeft);
  root->addStretch(1);
  return page;
}

QWidget *DisplayConfigWidget::CreateLayersPage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Display & topics"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to display_config in config.json: display_name, topic, visible. "
      "Labels on the left match layer names; edit the ROS topic for each layer."));

  QScrollArea *scroll = new QScrollArea(page);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  QWidget *scroll_body = new QWidget;
  QVBoxLayout *sl = new QVBoxLayout(scroll_body);
  sl->setContentsMargins(0, 0, 8, 0);
  sl->setSpacing(4);

  const std::vector<std::pair<QString, std::vector<std::pair<std::string, QString>>>> groups = {
      {QStringLiteral("Map & localization"),
       {{DISPLAY_MAP, QStringLiteral("Occupancy map")},
        {DISPLAY_ROBOT, QStringLiteral("Odometry / robot")}}},
      {QStringLiteral("Perception"),
       {{DISPLAY_LASER, QStringLiteral("Laser scan")}}},
      {QStringLiteral("Planning"),
       {{DISPLAY_GLOBAL_PATH, QStringLiteral("Global path")},
        {DISPLAY_LOCAL_PATH, QStringLiteral("Local path")}}},
      {QStringLiteral("Costmaps"),
       {{DISPLAY_GLOBAL_COST_MAP, QStringLiteral("Global costmap")},
        {DISPLAY_LOCAL_COST_MAP, QStringLiteral("Local costmap")}}},
      {QStringLiteral("Interaction"),
       {{DISPLAY_ROBOT_FOOTPRINT, QStringLiteral("Robot footprint")},
        {DISPLAY_GOAL, QStringLiteral("Navigation goal")}}},
  };

  for (const auto &grp : groups) {
    AddSectionHeader(sl, grp.first);
    QFrame *card = CreateSettingsCard(scroll_body);
    QVBoxLayout *card_layout = new QVBoxLayout(card);
    card_layout->setContentsMargins(0, 0, 0, 0);
    card_layout->setSpacing(0);

    for (size_t i = 0; i < grp.second.size(); ++i) {
      const auto &entry = grp.second[i];
      const std::string &display_name = entry.first;

      QWidget *row = new QWidget(card);
      row->setStyleSheet(QStringLiteral("QWidget { background:transparent; }"));
      QHBoxLayout *h = new QHBoxLayout(row);
      h->setContentsMargins(14, 12, 14, 12);
      h->setSpacing(12);

      QLabel *name = new QLabel(entry.second);
      name->setMinimumWidth(132);
      name->setStyleSheet(QStringLiteral("font-size:13px;color:#202124;font-weight:500;"));

      QLineEdit *topic_edit = new QLineEdit(row);
      topic_edit->setPlaceholderText(QStringLiteral("/topic/name"));
      topic_edit->setStyleSheet(LineEditStyle());
      display_topic_edits_[display_name] = topic_edit;
      connect(topic_edit, &QLineEdit::editingFinished, [this, display_name, topic_edit]() {
        OnDisplayTopicChanged(display_name, topic_edit->text());
      });

      QToolButton *toggle = new QToolButton(row);
      toggle->setCheckable(true);
      toggle->setChecked(true);
      toggle->setCursor(Qt::PointingHandCursor);
      toggle->setStyleSheet(ToggleStyle());
      display_toggle_buttons_[display_name] = toggle;
      connect(toggle, &QToolButton::toggled, [this, display_name](bool checked) {
        OnToggleDisplay(display_name, checked);
      });

      h->addWidget(name, 0);
      h->addWidget(topic_edit, 1);
      h->addWidget(toggle, 0);
      card_layout->addWidget(row);

      if (i + 1 < grp.second.size()) {
        QFrame *sep = new QFrame(card);
        sep->setFixedHeight(1);
        sep->setStyleSheet(QStringLiteral("background:rgba(0,0,0,0.06); border:none; max-height:1px;"));
        card_layout->addWidget(sep);
      }
    }
    sl->addWidget(card);
  }

  sl->addStretch();
  scroll->setWidget(scroll_body);
  root->addWidget(scroll, 1);
  return page;
}

QWidget *DisplayConfigWidget::CreateImagePage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Cameras"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to images in config.json: location (dock id), topic, enable."));

  QFrame *card = CreateSettingsCard(page);
  QVBoxLayout *card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(12, 12, 12, 12);
  card_layout->setSpacing(10);

  image_table_ = new QTableWidget(0, 4, card);
  image_table_->setHorizontalHeaderLabels(
      QStringList() << QStringLiteral("Location") << QStringLiteral("Topic") << QStringLiteral("Enable") << QStringLiteral(""));
  image_table_->horizontalHeader()->setStretchLastSection(true);
  image_table_->horizontalHeader()->setDefaultSectionSize(100);
  image_table_->verticalHeader()->setVisible(false);
  image_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  image_table_->setShowGrid(false);
  image_table_->setAlternatingRowColors(true);
  image_table_->setStyleSheet(
      QStringLiteral("QTableWidget { border:none; background:#fafafa; border-radius:8px; font-size:13px; }"
                     "QTableWidget::item { padding:8px 6px; border-bottom:1px solid rgba(0,0,0,0.05); }"
                     "QHeaderView::section { background:#f1f3f4; padding:8px; border:none; "
                     "border-bottom:1px solid rgba(0,0,0,0.08); font-size:12px; color:#5f6368; font-weight:600; }"));

  connect(image_table_, &QTableWidget::cellChanged, this, [this](int row, int) {
    OnImageConfigChanged(row);
  });

  QPushButton *add_btn = new QPushButton(QStringLiteral("Add camera"), card);
  add_btn->setCursor(Qt::PointingHandCursor);
  add_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:1px solid rgba(26,115,232,0.45); border-radius:8px; padding:8px 16px; "
                     "background:#fff; color:#1a73e8; font-weight:500; font-size:13px; }"
                     "QPushButton:hover { background:#e8f0fe; }"));
  connect(add_btn, &QPushButton::clicked, this, &DisplayConfigWidget::OnAddImageConfig);

  card_layout->addWidget(image_table_);
  card_layout->addWidget(add_btn, 0, Qt::AlignLeft);
  root->addWidget(card, 1);
  return page;
}

QWidget *DisplayConfigWidget::CreateRobotPage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Robot shape"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to robot_shape_config in config.json: shaped_points, is_ellipse, color, opacity."));

  QScrollArea *scroll = new QScrollArea(page);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  QWidget *scroll_content = new QWidget;
  QVBoxLayout *outer = new QVBoxLayout(scroll_content);
  outer->setContentsMargins(0, 0, 8, 0);
  outer->setSpacing(12);

  AddSectionHeader(outer, QStringLiteral("Polygon vertices"));
  QFrame *points_card = CreateSettingsCard(scroll_content);
  QVBoxLayout *points_layout = new QVBoxLayout(points_card);
  points_layout->setContentsMargins(14, 14, 14, 14);
  points_layout->setSpacing(10);

  QLabel *points_hint = new QLabel(QStringLiteral("Vertices in the plane (meters)"));
  points_hint->setStyleSheet(QStringLiteral("QLabel { color:#80868b; font-size:12px; }"));
  points_layout->addWidget(points_hint);

  robot_points_table_ = new QTableWidget(0, 2, points_card);
  robot_points_table_->setHorizontalHeaderLabels(QStringList() << QStringLiteral("X") << QStringLiteral("Y"));
  robot_points_table_->horizontalHeader()->setStretchLastSection(true);
  robot_points_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  robot_points_table_->setShowGrid(false);
  robot_points_table_->setStyleSheet(
      QStringLiteral("QTableWidget { border:none; background:#fafafa; border-radius:8px; }"
                     "QTableWidget::item { padding:6px; border-bottom:1px solid rgba(0,0,0,0.05); }"
                     "QHeaderView::section { background:#f1f3f4; padding:6px; border:none; font-size:12px; }"));

  connect(robot_points_table_, &QTableWidget::cellChanged, this, &DisplayConfigWidget::OnRobotShapePointChanged);

  QHBoxLayout *points_btn_layout = new QHBoxLayout();
  QPushButton *add_point_btn = new QPushButton(QStringLiteral("Add vertex"), points_card);
  add_point_btn->setCursor(Qt::PointingHandCursor);
  add_point_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:1px solid rgba(26,115,232,0.45); border-radius:8px; padding:6px 12px; "
                     "background:#fff; color:#1a73e8; font-size:13px; }"
                     "QPushButton:hover { background:#e8f0fe; }"));
  connect(add_point_btn, &QPushButton::clicked, [this]() {
    int row = robot_points_table_->rowCount();
    robot_points_table_->insertRow(row);
    robot_points_table_->setItem(row, 0, new QTableWidgetItem(QStringLiteral("0.0")));
    robot_points_table_->setItem(row, 1, new QTableWidgetItem(QStringLiteral("0.0")));
    OnRobotShapePointChanged();
  });

  QPushButton *remove_point_btn = new QPushButton(QStringLiteral("Remove selected"), points_card);
  remove_point_btn->setCursor(Qt::PointingHandCursor);
  remove_point_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:none; border-radius:8px; padding:6px 12px; background:transparent; "
                     "color:#d93025; font-size:13px; }"
                     "QPushButton:hover { background:#fce8e6; }"));
  connect(remove_point_btn, &QPushButton::clicked, [this]() {
    int row = robot_points_table_->currentRow();
    if (row >= 0) {
      robot_points_table_->removeRow(row);
      OnRobotShapePointChanged();
    }
  });

  points_btn_layout->addWidget(add_point_btn);
  points_btn_layout->addWidget(remove_point_btn);
  points_btn_layout->addStretch();

  points_layout->addWidget(robot_points_table_);
  points_layout->addLayout(points_btn_layout);
  outer->addWidget(points_card);

  AddSectionHeader(outer, QStringLiteral("Style"));
  QFrame *style_card = CreateSettingsCard(scroll_content);
  QVBoxLayout *style_layout = new QVBoxLayout(style_card);
  style_layout->setContentsMargins(14, 14, 14, 14);
  style_layout->setSpacing(14);

  robot_is_ellipse_checkbox_ = new QCheckBox(QStringLiteral("Approximate with ellipse"), style_card);
  robot_is_ellipse_checkbox_->setStyleSheet(QStringLiteral("QCheckBox { font-size:13px; color:#202124; spacing:8px; }"
                                                           "QCheckBox::indicator { width:18px; height:18px; }"));
  connect(robot_is_ellipse_checkbox_, &QCheckBox::toggled, this, &DisplayConfigWidget::OnRobotShapeIsEllipseChanged);

  QHBoxLayout *color_layout = new QHBoxLayout();
  QLabel *color_label = new QLabel(QStringLiteral("Color"));
  color_label->setFixedWidth(72);
  color_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  robot_color_button_ = new QPushButton(QStringLiteral("Choose color"), style_card);
  robot_color_button_->setMinimumWidth(120);
  robot_color_button_->setCursor(Qt::PointingHandCursor);
  robot_color_button_->setStyleSheet(
      QStringLiteral("QPushButton { border:1px solid #dadce0; border-radius:8px; padding:8px 12px; background:#fafafa; "
                     "font-size:13px; }"
                     "QPushButton:hover { border-color:#1a73e8; }"));
  connect(robot_color_button_, &QPushButton::clicked, this, &DisplayConfigWidget::OnRobotShapeColorChanged);
  color_layout->addWidget(color_label);
  color_layout->addWidget(robot_color_button_);
  color_layout->addStretch();

  QHBoxLayout *opacity_layout = new QHBoxLayout();
  QLabel *opacity_label = new QLabel(QStringLiteral("Opacity"));
  opacity_label->setFixedWidth(72);
  opacity_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  robot_opacity_slider_ = new QSlider(Qt::Horizontal, style_card);
  robot_opacity_slider_->setRange(0, 100);
  robot_opacity_slider_->setValue(50);
  robot_opacity_slider_->setStyleSheet(
      QStringLiteral("QSlider::groove:horizontal { height:6px; background:#e8eaed; border-radius:3px; }"
                     "QSlider::handle:horizontal { background:#1a73e8; width:18px; margin:-6px 0; border-radius:9px; }"));
  robot_opacity_label_ = new QLabel(QStringLiteral("50%"), style_card);
  robot_opacity_label_->setFixedWidth(44);
  robot_opacity_label_->setStyleSheet(QStringLiteral("color:#5f6368;font-size:13px;"));
  connect(robot_opacity_slider_, &QSlider::valueChanged, [this](int value) {
    robot_opacity_label_->setText(QString::number(value) + QStringLiteral("%"));
    OnRobotShapeOpacityChanged(value);
  });
  opacity_layout->addWidget(opacity_label);
  opacity_layout->addWidget(robot_opacity_slider_, 1);
  opacity_layout->addWidget(robot_opacity_label_);

  style_layout->addWidget(robot_is_ellipse_checkbox_);
  style_layout->addLayout(color_layout);
  style_layout->addLayout(opacity_layout);

  outer->addWidget(style_card);
  outer->addStretch();

  scroll->setWidget(scroll_content);
  root->addWidget(scroll, 1);
  return page;
}

QWidget *DisplayConfigWidget::CreateMapPage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Default map"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to map_config.path in config.json: YAML file to load on startup (with or without .yaml)."));

  QFrame *card = CreateSettingsCard(page);
  QVBoxLayout *card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(16, 16, 16, 16);
  card_layout->setSpacing(12);

  QHBoxLayout *path_layout = new QHBoxLayout();
  QLabel *path_label = new QLabel(QStringLiteral("Map path"));
  path_label->setFixedWidth(88);
  path_label->setStyleSheet(QStringLiteral("color:#3c4043;font-size:13px;"));
  map_path_edit_ = new QLineEdit(card);
  map_path_edit_->setPlaceholderText(QStringLiteral("e.g. C:/maps/office.yaml"));
  map_path_edit_->setStyleSheet(LineEditStyle());
  connect(map_path_edit_, &QLineEdit::editingFinished, [this]() {
    if (is_loading_config_) {
      return;
    }
    Config::ConfigManager::Instance()->GetRootConfig().map_config.path = map_path_edit_->text().toStdString();
    AutoSaveConfig();
  });
  QPushButton *browse_btn = new QPushButton(QStringLiteral("Browse…"), card);
  browse_btn->setCursor(Qt::PointingHandCursor);
  browse_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:1px solid #dadce0; border-radius:8px; padding:8px 14px; background:#fff; "
                     "font-size:13px; color:#1a73e8; }"
                     "QPushButton:hover { background:#e8f0fe; border-color:#1a73e8; }"));
  connect(browse_btn, &QPushButton::clicked, [this]() {
    QString f = QFileDialog::getOpenFileName(this, QStringLiteral("Select map YAML"), QString(),
                                               QStringLiteral("YAML (*.yaml *.yml);;All (*.*)"));
    if (!f.isEmpty()) {
      map_path_edit_->setText(f);
      if (!is_loading_config_) {
        Config::ConfigManager::Instance()->GetRootConfig().map_config.path = f.toStdString();
        AutoSaveConfig();
      }
    }
  });
  path_layout->addWidget(path_label);
  path_layout->addWidget(map_path_edit_, 1);
  path_layout->addWidget(browse_btn);
  card_layout->addLayout(path_layout);

  root->addWidget(card);
  root->addStretch(1);
  return page;
}

QWidget *DisplayConfigWidget::CreateKeyValuePage() {
  QWidget *page = new QWidget;
  QVBoxLayout *root = new QVBoxLayout(page);
  root->setContentsMargins(8, 4, 8, 8);
  root->setSpacing(0);

  QLabel *page_title = new QLabel(QStringLiteral("Key-value"));
  page_title->setObjectName(QStringLiteral("pageTitle"));
  root->addWidget(page_title);
  AddHintLabel(root, QStringLiteral(
      "Maps to key_value in config.json: arbitrary string pairs for channel and app options "
      "(e.g. BaseFrameId, DiagnosticMsgType)."));

  QFrame *card = CreateSettingsCard(page);
  QVBoxLayout *card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(0, 8, 0, 8);
  card_layout->setSpacing(0);

  QScrollArea *scroll = new QScrollArea(card);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setMinimumHeight(200);
  key_value_host_ = new QWidget;
  key_value_layout_ = new QVBoxLayout(key_value_host_);
  key_value_layout_->setContentsMargins(0, 0, 0, 0);
  key_value_layout_->setSpacing(0);
  scroll->setWidget(key_value_host_);
  card_layout->addWidget(scroll);

  QPushButton *add_btn = new QPushButton(QStringLiteral("Add entry"), page);
  add_btn->setCursor(Qt::PointingHandCursor);
  add_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:1px solid rgba(26,115,232,0.45); border-radius:8px; padding:8px 16px; "
                     "background:#fff; color:#1a73e8; font-weight:500; margin-top:10px; }"
                     "QPushButton:hover { background:#e8f0fe; }"));
  connect(add_btn, &QPushButton::clicked, this, &DisplayConfigWidget::OnAddKeyValue);

  root->addWidget(card, 1);
  root->addWidget(add_btn, 0, Qt::AlignLeft);
  return page;
}

void DisplayConfigWidget::SetChannelList(const std::vector<std::string> &channel_list) {
  channel_list_ = channel_list;

  channel_type_combo_->blockSignals(true);
  channel_type_combo_->clear();

  channel_type_combo_->addItem(QStringLiteral("Auto"), QStringLiteral("auto"));
  for (const auto &channel_type : channel_list_) {
    QString q = QString::fromStdString(channel_type);
    channel_type_combo_->addItem(q, q);
  }

  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  std::string channel_type =
      config.channel_config.channel_type.empty() ? "auto" : config.channel_config.channel_type;
  int index = channel_type_combo_->findData(QString::fromStdString(channel_type));
  if (index >= 0) {
    channel_type_combo_->setCurrentIndex(index);
  } else {
    channel_type_combo_->setCurrentIndex(0);
  }

  channel_type_combo_->blockSignals(false);
}

void DisplayConfigWidget::SetDisplayManager(Display::DisplayManager *manager) {
  display_manager_ = manager;
  if (display_manager_) {
    LoadConfig();
  }
}

void DisplayConfigWidget::OnToggleDisplay(const std::string &display_name, bool visible) {
  UpdateDisplayVisibility(display_name, visible);
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  auto it = std::find_if(config.display_config.begin(), config.display_config.end(),
                         [&display_name](const auto &item) { return item.display_name == display_name; });
  auto topic_it = display_topic_edits_.find(display_name);
  std::string topic =
      (topic_it != display_topic_edits_.end()) ? topic_it->second->text().toStdString() : std::string();
  if (it != config.display_config.end()) {
    it->visible = visible;
  } else {
    config.display_config.push_back(Config::DisplayConfig(display_name, topic, visible));
  }
  AutoSaveConfig();
}

void DisplayConfigWidget::OnDisplayTopicChanged(const std::string &display_name, const QString &topic) {
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  auto it = std::find_if(config.display_config.begin(), config.display_config.end(),
                         [&display_name](const auto &item) { return item.display_name == display_name; });
  if (it != config.display_config.end()) {
    it->topic = topic.toStdString();
  } else {
    auto toggle_it = display_toggle_buttons_.find(display_name);
    bool vis = (toggle_it != display_toggle_buttons_.end()) ? toggle_it->second->isChecked() : true;
    config.display_config.push_back(Config::DisplayConfig(display_name, topic.toStdString(), vis));
  }
  AutoSaveConfig();
}

void DisplayConfigWidget::OnKeyValueChanged(const std::string &key, const QString &value) {
  SET_KEY_VALUE(key, value.toStdString())
}

void DisplayConfigWidget::OnAddKeyValue() {
  bool ok = false;
  QString key = QInputDialog::getText(this, QStringLiteral("Add entry"), QStringLiteral("Key:"),
                                        QLineEdit::Normal, QString(), &ok);
  if (ok && !key.isEmpty()) {
    QString value = QInputDialog::getText(this, QStringLiteral("Add entry"), QStringLiteral("Value:"),
                                            QLineEdit::Normal, QString(), &ok);
    if (ok) {
      SET_KEY_VALUE(key.toStdString(), value.toStdString())
      RefreshKeyValueTab();
    }
  }
}

void DisplayConfigWidget::OnRemoveKeyValue(const std::string &key) {
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  config.key_value.erase(key);
  AutoSaveConfig();
  RefreshKeyValueTab();
}

void DisplayConfigWidget::RefreshKeyValueTab() {
  if (!key_value_layout_) {
    return;
  }
  QLayoutItem *item = nullptr;
  while ((item = key_value_layout_->takeAt(0)) != nullptr) {
    if (item->widget()) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  key_value_edits_.clear();

  auto &config = Config::ConfigManager::Instance()->GetRootConfig();

  for (const auto &[key, value] : config.key_value) {
    QWidget *item_widget = new QWidget(key_value_host_);
    item_widget->setStyleSheet(QStringLiteral("QWidget { background:#fff; border-bottom:1px solid rgba(0,0,0,0.06); }"));

    QHBoxLayout *item_layout = new QHBoxLayout(item_widget);
    item_layout->setContentsMargins(14, 12, 14, 12);
    item_layout->setSpacing(12);

    QLabel *key_label = new QLabel(QString::fromStdString(key), item_widget);
    key_label->setFixedWidth(128);
    key_label->setStyleSheet(QStringLiteral("font-size:13px;color:#202124;font-weight:500;"));
    item_layout->addWidget(key_label);

    QLineEdit *value_edit = new QLineEdit(QString::fromStdString(value), item_widget);
    value_edit->setPlaceholderText(QStringLiteral("Value"));
    value_edit->setStyleSheet(LineEditStyle());
    key_value_edits_[key] = value_edit;
    connect(value_edit, &QLineEdit::editingFinished, [this, key, value_edit]() {
      OnKeyValueChanged(key, value_edit->text());
    });
    item_layout->addWidget(value_edit, 1);

    QPushButton *remove_btn = new QPushButton(QStringLiteral("Remove"), item_widget);
    remove_btn->setFixedWidth(52);
    remove_btn->setCursor(Qt::PointingHandCursor);
    remove_btn->setStyleSheet(
        QStringLiteral("QPushButton { border:none; border-radius:6px; padding:6px 8px; background:transparent; "
                       "color:#d93025; font-size:13px; }"
                       "QPushButton:hover { background:#fce8e6; }"));
    connect(remove_btn, &QPushButton::clicked, [this, key]() { OnRemoveKeyValue(key); });
    item_layout->addWidget(remove_btn);

    key_value_layout_->addWidget(item_widget);
  }

  key_value_layout_->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
}

void DisplayConfigWidget::OnAddImageConfig() {
  int row = image_table_->rowCount();
  image_table_->insertRow(row);

  QTableWidgetItem *location_item = new QTableWidgetItem(QString());
  location_item->setToolTip(QStringLiteral("Dock id, e.g. front, rear"));
  QTableWidgetItem *topic_item = new QTableWidgetItem(QString());
  topic_item->setToolTip(QStringLiteral("Image topic, e.g. /camera/front/image_raw"));
  QTableWidgetItem *enable_item = new QTableWidgetItem(QStringLiteral("true"));
  enable_item->setFlags(enable_item->flags() & ~Qt::ItemIsEditable);

  QCheckBox *enable_checkbox = new QCheckBox();
  enable_checkbox->setChecked(true);
  enable_checkbox->setStyleSheet(
      QStringLiteral("QCheckBox::indicator { width:18px; height:18px; }"));
  connect(enable_checkbox, &QCheckBox::toggled, [this, row](bool checked) {
    image_table_->item(row, 2)->setText(checked ? QStringLiteral("true") : QStringLiteral("false"));
    OnImageConfigChanged(row);
  });

  QPushButton *remove_btn = new QPushButton(QStringLiteral("Remove"));
  remove_btn->setCursor(Qt::PointingHandCursor);
  remove_btn->setStyleSheet(
      QStringLiteral("QPushButton { border:none; color:#d93025; font-size:13px; padding:4px 8px; }"
                     "QPushButton:hover { background:#fce8e6; border-radius:6px; }"));
  connect(remove_btn, &QPushButton::clicked, [this, row]() { OnRemoveImageConfig(row); });

  image_table_->setItem(row, 0, location_item);
  image_table_->setItem(row, 1, topic_item);
  image_table_->setItem(row, 2, enable_item);
  image_table_->setCellWidget(row, 2, enable_checkbox);
  image_table_->setCellWidget(row, 3, remove_btn);

  OnImageConfigChanged(row);
}

void DisplayConfigWidget::OnRemoveImageConfig(int row) {
  image_table_->removeRow(row);

  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  if (row < static_cast<int>(config.images.size())) {
    config.images.erase(config.images.begin() + row);
    AutoSaveConfig();
  }

  for (int i = row; i < image_table_->rowCount(); i++) {
    QPushButton *btn = qobject_cast<QPushButton *>(image_table_->cellWidget(i, 3));
    if (btn) {
      btn->disconnect();
      connect(btn, &QPushButton::clicked, [this, i]() { OnRemoveImageConfig(i); });
    }
    QCheckBox *checkbox = qobject_cast<QCheckBox *>(image_table_->cellWidget(i, 2));
    if (checkbox) {
      checkbox->disconnect();
      connect(checkbox, &QCheckBox::toggled, [this, i](bool checked) {
        image_table_->item(i, 2)->setText(checked ? QStringLiteral("true") : QStringLiteral("false"));
        OnImageConfigChanged(i);
      });
    }
  }
}

void DisplayConfigWidget::OnImageConfigChanged(int row) {
  if (row < 0 || row >= image_table_->rowCount()) {
    return;
  }

  auto &config = Config::ConfigManager::Instance()->GetRootConfig();

  QTableWidgetItem *location_item = image_table_->item(row, 0);
  QTableWidgetItem *topic_item = image_table_->item(row, 1);
  QCheckBox *enable_checkbox = qobject_cast<QCheckBox *>(image_table_->cellWidget(row, 2));

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
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  config.robot_shape_config.shaped_points.clear();

  for (int row = 0; row < robot_points_table_->rowCount(); row++) {
    QTableWidgetItem *x_item = robot_points_table_->item(row, 0);
    QTableWidgetItem *y_item = robot_points_table_->item(row, 1);

    if (x_item && y_item) {
      bool x_ok = false;
      bool y_ok = false;
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
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  config.robot_shape_config.is_ellipse = checked;
  AutoSaveConfig();
}

void DisplayConfigWidget::OnRobotShapeColorChanged() {
  QColor color = QColorDialog::getColor(robot_color_, this, QStringLiteral("Choose color"));
  if (color.isValid()) {
    robot_color_ = color;
    QString color_style = QStringLiteral("background-color: %1;").arg(color.name());
    robot_color_button_->setStyleSheet(
        QStringLiteral("QPushButton { border:1px solid #dadce0; border-radius:8px; padding:8px 12px; }"
                       "QPushButton:hover { border-color:#1a73e8; }") +
        color_style);

    auto &config = Config::ConfigManager::Instance()->GetRootConfig();
    QString color_str = QStringLiteral("0x%1").arg(color.rgb(), 8, 16, QChar('0')).toUpper();
    config.robot_shape_config.color = color_str.toStdString();
    AutoSaveConfig();
  }
}

void DisplayConfigWidget::OnRobotShapeOpacityChanged(int value) {
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
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
  Config::ConfigManager::Instance()->StoreConfig();
}

void DisplayConfigWidget::LoadConfig() {
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();

  for (auto &display_config : config.display_config) {
    auto toggle_it = display_toggle_buttons_.find(display_config.display_name);
    if (toggle_it != display_toggle_buttons_.end()) {
      toggle_it->second->blockSignals(true);
      toggle_it->second->setChecked(display_config.visible);
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
    location_item->setToolTip(QStringLiteral("Location id"));
    QTableWidgetItem *topic_item = new QTableWidgetItem(QString::fromStdString(image_config.topic));
    topic_item->setToolTip(QStringLiteral("Image topic"));
    QTableWidgetItem *enable_item = new QTableWidgetItem(image_config.enable ? QStringLiteral("true") : QStringLiteral("false"));
    enable_item->setFlags(enable_item->flags() & ~Qt::ItemIsEditable);

    QCheckBox *enable_checkbox = new QCheckBox();
    enable_checkbox->setChecked(image_config.enable);
    connect(enable_checkbox, &QCheckBox::toggled, [this, row](bool checked) {
      image_table_->item(row, 2)->setText(checked ? QStringLiteral("true") : QStringLiteral("false"));
      OnImageConfigChanged(row);
    });

    QPushButton *remove_btn = new QPushButton(QStringLiteral("Remove"));
    remove_btn->setCursor(Qt::PointingHandCursor);
    remove_btn->setStyleSheet(
        QStringLiteral("QPushButton { border:none; color:#d93025; font-size:13px; padding:4px 8px; }"
                       "QPushButton:hover { background:#fce8e6; border-radius:6px; }"));
    connect(remove_btn, &QPushButton::clicked, [this, row]() { OnRemoveImageConfig(row); });

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
  if (color_str.startsWith(QStringLiteral("0x"))) {
    bool ok = false;
    uint rgb = color_str.mid(2).toUInt(&ok, 16);
    if (ok) {
      robot_color_ = QColor::fromRgb(rgb);
      QString color_style = QStringLiteral("background-color: %1;").arg(robot_color_.name());
      robot_color_button_->setStyleSheet(
          QStringLiteral("QPushButton { border:1px solid #dadce0; border-radius:8px; padding:8px 12px; }"
                         "QPushButton:hover { border-color:#1a73e8; }") +
          color_style);
    }
  }

  robot_opacity_slider_->blockSignals(true);
  robot_opacity_slider_->setValue(static_cast<int>(config.robot_shape_config.opacity * 100));
  robot_opacity_label_->setText(QString::number(robot_opacity_slider_->value()) + QStringLiteral("%"));
  robot_opacity_slider_->blockSignals(false);

  is_loading_config_ = true;

  std::string channel_type =
      config.channel_config.channel_type.empty() ? "auto" : config.channel_config.channel_type;
  channel_type_combo_->blockSignals(true);
  int index = channel_type_combo_->findData(QString::fromStdString(channel_type));
  if (index >= 0) {
    channel_type_combo_->setCurrentIndex(index);
  } else {
    channel_type_combo_->setCurrentIndex(0);
  }
  channel_type_combo_->blockSignals(false);

  std::string rosbridge_ip =
      config.channel_config.rosbridge_config.ip.empty() ? "127.0.0.1" : config.channel_config.rosbridge_config.ip;
  rosbridge_ip_edit_->blockSignals(true);
  rosbridge_ip_edit_->setText(QString::fromStdString(rosbridge_ip));
  rosbridge_ip_edit_->blockSignals(false);

  std::string rosbridge_port =
      config.channel_config.rosbridge_config.port.empty() ? "9090" : config.channel_config.rosbridge_config.port;
  rosbridge_port_edit_->blockSignals(true);
  rosbridge_port_edit_->setText(QString::fromStdString(rosbridge_port));
  rosbridge_port_edit_->blockSignals(false);

  if (map_path_edit_) {
    map_path_edit_->blockSignals(true);
    map_path_edit_->setText(QString::fromStdString(config.map_config.path));
    map_path_edit_->blockSignals(false);
  }

  bool show_rosbridge = (channel_type == "rosbridge");
  rosbridge_ip_edit_->setEnabled(show_rosbridge);
  rosbridge_port_edit_->setEnabled(show_rosbridge);

  is_loading_config_ = false;
}

void DisplayConfigWidget::SaveConfig() {
  AutoSaveConfig();
}
