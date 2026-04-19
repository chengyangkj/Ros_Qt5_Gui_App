#pragma once

#include <QWidget>
#include "msg/diagnostic_snapshot.h"

class QLabel;
class QLineEdit;
class QPushButton;
class QTreeWidget;
class QButtonGroup;

class DiagnosticDockWidget : public QWidget {
  Q_OBJECT

 public:
  explicit DiagnosticDockWidget(QWidget *parent = nullptr);

  void SetSnapshot(const basic::DiagnosticSnapshot &snapshot);

 private:
  void RebuildUi();
  void UpdateSummary();
  static QString FormatTimeMs(int64_t ms);
  static QString LevelDisplayName(int level);
  static QColor LevelColor(int level);

  basic::DiagnosticSnapshot snapshot_;
  QString search_lower_;
  int filter_level_{-1};

  QLabel *summary_ok_{nullptr};
  QLabel *summary_warn_{nullptr};
  QLabel *summary_error_{nullptr};
  QLabel *summary_stale_{nullptr};
  QLineEdit *search_edit_{nullptr};
  QButtonGroup *filter_group_{nullptr};
  QLabel *empty_label_{nullptr};
  QLabel *filter_hint_{nullptr};
  QTreeWidget *tree_{nullptr};
};
