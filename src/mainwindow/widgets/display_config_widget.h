#pragma once
#include <QWidget>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QGroupBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QColorDialog>
#include <QFileDialog>
#include <QTableWidget>
#include <QHeaderView>
#include <QComboBox>
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
  void InitDisplayConfigTab();
  void InitKeyValueTab();
  void InitImageConfigTab();
  void InitRobotShapeTab();
  void InitChannelConfigTab();
  void RefreshKeyValueTab();
  void UpdateDisplayVisibility(const std::string &display_name, bool visible);
  void AutoSaveConfig();
  
  Display::DisplayManager *display_manager_{nullptr};
  QVBoxLayout *main_layout_{nullptr};
  QTabWidget *tab_widget_{nullptr};
  
  QWidget *display_tab_{nullptr};
  QScrollArea *display_scroll_area_{nullptr};
  QWidget *display_scroll_content_{nullptr};
  std::map<std::string, QToolButton*> display_toggle_buttons_;
  std::map<std::string, QLineEdit*> display_topic_edits_;
  
  QWidget *key_value_tab_{nullptr};
  QScrollArea *key_value_scroll_area_{nullptr};
  QWidget *key_value_scroll_content_{nullptr};
  std::map<std::string, QLineEdit*> key_value_edits_;
  
  QWidget *image_tab_{nullptr};
  QTableWidget *image_table_{nullptr};
  
  QWidget *robot_shape_tab_{nullptr};
  QTableWidget *robot_points_table_{nullptr};
  QCheckBox *robot_is_ellipse_checkbox_{nullptr};
  QPushButton *robot_color_button_{nullptr};
  QSlider *robot_opacity_slider_{nullptr};
  QLabel *robot_opacity_label_{nullptr};
  QColor robot_color_;
  
  QWidget *channel_config_tab_{nullptr};
  QComboBox *channel_type_combo_{nullptr};
  QLineEdit *rosbridge_ip_edit_{nullptr};
  QLineEdit *rosbridge_port_edit_{nullptr};
  QPushButton *reconnect_channel_btn_{nullptr};
  bool is_loading_config_{false};
};

