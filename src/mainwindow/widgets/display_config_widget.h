#pragma once
#include <QWidget>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QListWidget>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QSlider>
#include <QColorDialog>
#include <QTableWidget>
#include <QHeaderView>
#include <QComboBox>
#include <QFrame>
#include <map>
#include <string>
#include <vector>

namespace Display {
class DisplayManager;
}

class DisplayConfigWidget : public QWidget {
  Q_OBJECT

 public:
  explicit DisplayConfigWidget(QWidget *parent = nullptr);
  ~DisplayConfigWidget();

  void SetDisplayManager(Display::DisplayManager *manager);
  void SetChannelList(const std::vector<std::string> &channel_list);
  void LoadConfig();
  void SaveConfig();

 private slots:
  void OnToggleDisplay(const std::string &display_name, bool visible);
  void OnDisplayTopicChanged(const std::string &display_name, const QString &topic);
  void OnKeyValueChanged(const std::string &key, const QString &value);
  void OnAddKeyValue();
  void OnRemoveKeyValue(const std::string &key);
  void OnAddImageConfig();
  void OnRemoveImageConfig(int row);
  void OnImageConfigChanged(int row);
  void OnRobotShapePointChanged();
  void OnRobotShapeIsEllipseChanged(bool checked);
  void OnRobotShapeColorChanged();
  void OnRobotShapeOpacityChanged(int value);

 private:
  void InitUI();
  void ApplyGlobalStyle();
  QWidget *CreateChannelPage();
  QWidget *CreateLayersPage();
  QWidget *CreateImagePage();
  QWidget *CreateRobotPage();
  QWidget *CreateMapPage();
  QWidget *CreateKeyValuePage();
  void RefreshKeyValueTab();
  void UpdateDisplayVisibility(const std::string &display_name, bool visible);
  void AutoSaveConfig();

  static QFrame *CreateSettingsCard(QWidget *parent);
  static QLabel *AddSectionHeader(QVBoxLayout *layout, const QString &title);
  static QLabel *AddHintLabel(QVBoxLayout *layout, const QString &text);

  Display::DisplayManager *display_manager_{nullptr};
  QVBoxLayout *main_layout_{nullptr};
  QListWidget *nav_list_{nullptr};
  QStackedWidget *page_stack_{nullptr};

  std::map<std::string, QToolButton *> display_toggle_buttons_;
  std::map<std::string, QLineEdit *> display_topic_edits_;

  QWidget *key_value_host_{nullptr};
  QVBoxLayout *key_value_layout_{nullptr};
  std::map<std::string, QLineEdit *> key_value_edits_;

  QTableWidget *image_table_{nullptr};

  QTableWidget *robot_points_table_{nullptr};
  QCheckBox *robot_is_ellipse_checkbox_{nullptr};
  QPushButton *robot_color_button_{nullptr};
  QSlider *robot_opacity_slider_{nullptr};
  QLabel *robot_opacity_label_{nullptr};
  QColor robot_color_;

  QComboBox *channel_type_combo_{nullptr};
  QLineEdit *rosbridge_ip_edit_{nullptr};
  QLineEdit *rosbridge_port_edit_{nullptr};
  QPushButton *reconnect_channel_btn_{nullptr};
  QLineEdit *map_path_edit_{nullptr};

  QLabel *title_label_{nullptr};
  QLabel *connection_section_label_{nullptr};
  QLabel *channel_type_label_{nullptr};
  QLabel *rosbridge_section_label_{nullptr};
  QLabel *rosbridge_ip_label_{nullptr};
  QLabel *rosbridge_port_label_{nullptr};
  QLabel *map_path_label_{nullptr};
  QPushButton *map_browse_btn_{nullptr};
  QPushButton *image_add_btn_{nullptr};
  QPushButton *key_value_add_btn_{nullptr};
  QLabel *robot_points_hint_label_{nullptr};
  QLabel *robot_polygon_section_label_{nullptr};
  QLabel *robot_style_section_label_{nullptr};
  QLabel *robot_color_caption_label_{nullptr};
  QLabel *robot_opacity_caption_label_{nullptr};
  QPushButton *robot_add_vertex_btn_{nullptr};
  QPushButton *robot_remove_vertex_btn_{nullptr};

  bool is_loading_config_{false};
  std::vector<std::string> channel_list_;
};
