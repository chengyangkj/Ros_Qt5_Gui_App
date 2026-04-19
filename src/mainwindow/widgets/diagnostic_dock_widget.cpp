#include "diagnostic_dock_widget.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QColor>
#include <QDateTime>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>

namespace {

bool StringContainsInsensitive(const std::string &s, const QString &q) {
  return QString::fromStdString(s).toLower().contains(q);
}

bool ComponentMatchesSearch(const std::string &hardware_id,
                             const std::string &component_name,
                             const basic::DiagnosticComponentState &st,
                             const QString &q) {
  if (q.isEmpty()) {
    return true;
  }
  if (StringContainsInsensitive(hardware_id, q)) {
    return true;
  }
  if (StringContainsInsensitive(component_name, q)) {
    return true;
  }
  if (StringContainsInsensitive(st.message, q)) {
    return true;
  }
  for (const auto &kv : st.key_values) {
    if (StringContainsInsensitive(kv.first, q) || StringContainsInsensitive(kv.second, q)) {
      return true;
    }
  }
  return false;
}

bool HardwareRowMatches(const std::string &hardware_id,
                        const std::map<std::string, basic::DiagnosticComponentState> &states,
                        const QString &search_q, int filter_level) {
  if (!search_q.isEmpty()) {
    bool any = false;
    if (StringContainsInsensitive(hardware_id, search_q)) {
      any = true;
    } else {
      for (const auto &e : states) {
        if (ComponentMatchesSearch(hardware_id, e.first, e.second, search_q)) {
          any = true;
          break;
        }
      }
    }
    if (!any) {
      return false;
    }
  }
  if (filter_level != -1) {
    bool has = false;
    for (const auto &e : states) {
      if (e.second.level == filter_level) {
        has = true;
        break;
      }
    }
    if (!has) {
      return false;
    }
  }
  return true;
}

std::map<std::string, basic::DiagnosticComponentState> FilterComponents(
    const std::string &hardware_id,
    const std::map<std::string, basic::DiagnosticComponentState> &states,
    const QString &search_q, int filter_level) {
  if (search_q.isEmpty() && filter_level == -1) {
    return states;
  }
  std::map<std::string, basic::DiagnosticComponentState> out;
  for (const auto &e : states) {
    if (!ComponentMatchesSearch(hardware_id, e.first, e.second, search_q)) {
      continue;
    }
    if (filter_level != -1 && e.second.level != filter_level) {
      continue;
    }
    out.insert(e);
  }
  return out;
}

int MaxLevelInMap(const std::map<std::string, basic::DiagnosticComponentState> &m) {
  int max_level = 0;
  for (const auto &e : m) {
    if (e.second.level > max_level) {
      max_level = e.second.level;
    }
  }
  return max_level;
}

int64_t LatestUpdateMs(const std::map<std::string, basic::DiagnosticComponentState> &m) {
  int64_t t = 0;
  for (const auto &e : m) {
    if (e.second.last_update_ms > t) {
      t = e.second.last_update_ms;
    }
  }
  return t;
}

}  // namespace

DiagnosticDockWidget::DiagnosticDockWidget(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto *summary_row = new QHBoxLayout();
  summary_ok_ = new QLabel();
  summary_warn_ = new QLabel();
  summary_error_ = new QLabel();
  summary_stale_ = new QLabel();
  for (auto *lb : {summary_ok_, summary_warn_, summary_error_, summary_stale_}) {
    lb->setStyleSheet(QStringLiteral("padding:4px 8px;border-radius:10px;font-size:11px;"));
  }
  summary_ok_->setStyleSheet(summary_ok_->styleSheet() +
                               QStringLiteral("background-color:rgba(46,125,50,0.15);color:#2e7d32;"));
  summary_warn_->setStyleSheet(summary_warn_->styleSheet() +
                               QStringLiteral("background-color:rgba(245,124,0,0.15);color:#f57c00;"));
  summary_error_->setStyleSheet(summary_error_->styleSheet() +
                                QStringLiteral("background-color:rgba(211,47,47,0.15);color:#d32f2f;"));
  summary_stale_->setStyleSheet(summary_stale_->styleSheet() +
                                QStringLiteral("background-color:rgba(97,97,97,0.15);color:#616161;"));
  summary_row->addWidget(summary_ok_);
  summary_row->addWidget(summary_warn_);
  summary_row->addWidget(summary_error_);
  summary_row->addWidget(summary_stale_);
  summary_row->addStretch();
  refresh_btn_ = new QPushButton(tr("Refresh"));
  refresh_btn_->setFixedHeight(26);
  connect(refresh_btn_, &QPushButton::clicked, this, [this]() { RebuildUi(); });
  summary_row->addWidget(refresh_btn_);
  root->addLayout(summary_row);

  auto *filter_row = new QHBoxLayout();
  search_edit_ = new QLineEdit();
  search_edit_->setPlaceholderText(tr("Search components, messages or key values…"));
  search_edit_->setClearButtonEnabled(true);
  filter_row->addWidget(search_edit_, 2);

  filter_group_ = new QButtonGroup(this);
  filter_group_->setExclusive(true);
  const struct {
    const char *label;
    int level;
  } chips[] = {{"All", -1}, {"OK", 0}, {"Warning", 1}, {"Error", 2}, {"Stale", 3}};
  auto *chip_layout = new QHBoxLayout();
  chip_layout->setSpacing(4);
  for (int i = 0; i < 5; ++i) {
    auto *b = new QPushButton(tr(chips[i].label));
    filter_chip_buttons_[i] = b;
    b->setCheckable(true);
    b->setFixedHeight(26);
    b->setProperty("diagLevel", chips[i].level);
    b->setStyleSheet(QStringLiteral(
        "QPushButton{padding:2px 8px;border-radius:10px;border:1px solid #ccc;background:#fff;}"
        "QPushButton:checked{font-weight:bold;background:#e3f2fd;border-color:#1976d2;}"));
    filter_group_->addButton(b);
    chip_layout->addWidget(b);
    if (chips[i].level == -1) {
      b->setChecked(true);
    }
  }
  filter_row->addLayout(chip_layout, 3);

  clear_filter_btn_ = new QPushButton(tr("Clear filter"));
  clear_filter_btn_->setFixedHeight(26);
  connect(clear_filter_btn_, &QPushButton::clicked, this, [this]() {
    search_edit_->clear();
    search_lower_.clear();
    filter_level_ = -1;
    for (QAbstractButton *ab : filter_group_->buttons()) {
      if (ab->property("diagLevel").toInt() == -1) {
        ab->setChecked(true);
        break;
      }
    }
    RebuildUi();
  });
  filter_row->addWidget(clear_filter_btn_);
  root->addLayout(filter_row);

  filter_hint_ = new QLabel();
  filter_hint_->setWordWrap(true);
  filter_hint_->setStyleSheet(QStringLiteral("color:#1565c0;font-size:11px;"));
  filter_hint_->hide();
  root->addWidget(filter_hint_);

  tree_ = new QTreeWidget();
  tree_->setColumnCount(2);
  tree_->setHeaderLabels({tr("Name / key"), tr("Status / value")});
  tree_->setAlternatingRowColors(true);
  tree_->setUniformRowHeights(false);
  root->addWidget(tree_, 1);

  empty_label_ = new QLabel(tr("No diagnostic data"));
  empty_label_->setAlignment(Qt::AlignCenter);
  empty_label_->setStyleSheet(QStringLiteral("color:#888;font-size:13px;padding:24px;"));
  empty_label_->hide();
  root->addWidget(empty_label_);

  connect(search_edit_, &QLineEdit::textChanged, this, [this](const QString &t) {
    search_lower_ = t.toLower();
    RebuildUi();
  });
  connect(filter_group_, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked), this,
          [this](QAbstractButton *b) {
            filter_level_ = b->property("diagLevel").toInt();
            RebuildUi();
          });

  UpdateSummary();
}

QString DiagnosticDockWidget::FormatTimeMs(int64_t ms) {
  if (ms <= 0) {
    return QStringLiteral("-");
  }
  QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
  return dt.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
}

QString DiagnosticDockWidget::LevelDisplayName(int level) const {
  switch (level) {
    case 0:
      return tr("OK");
    case 1:
      return tr("Warning");
    case 2:
      return tr("Error");
    case 3:
      return tr("Stale");
    default:
      return tr("Unknown");
  }
}

QColor DiagnosticDockWidget::LevelColor(int level) {
  switch (level) {
    case 0:
      return QColor(QStringLiteral("#2e7d32"));
    case 1:
      return QColor(QStringLiteral("#f57c00"));
    case 2:
      return QColor(QStringLiteral("#d32f2f"));
    case 3:
      return QColor(QStringLiteral("#616161"));
    default:
      return QColor(QStringLiteral("#333333"));
  }
}

void DiagnosticDockWidget::SetSnapshot(const basic::DiagnosticSnapshot &snapshot) {
  snapshot_ = snapshot;
  UpdateSummary();
  RebuildUi();
}

void DiagnosticDockWidget::UpdateSummary() {
  int c0 = 0, c1 = 0, c2 = 0, c3 = 0;
  for (const auto &hw : snapshot_.hardware) {
    for (const auto &comp : hw.second) {
      int lv = comp.second.level;
      if (lv < 0 || lv > 3) {
        continue;
      }
      if (lv == 0) {
        ++c0;
      } else if (lv == 1) {
        ++c1;
      } else if (lv == 2) {
        ++c2;
      } else {
        ++c3;
      }
    }
  }
  summary_ok_->setText(tr("%1 OK").arg(c0));
  summary_warn_->setText(tr("%1 Warning").arg(c1));
  summary_error_->setText(tr("%1 Error").arg(c2));
  summary_stale_->setText(tr("%1 Stale").arg(c3));
}

void DiagnosticDockWidget::RebuildUi() {
  tree_->clear();
  const bool has_filter = !search_lower_.isEmpty() || filter_level_ != -1;

  std::vector<std::pair<std::string, std::map<std::string, basic::DiagnosticComponentState>>> rows;
  for (const auto &hw : snapshot_.hardware) {
    if (!HardwareRowMatches(hw.first, hw.second, search_lower_, filter_level_)) {
      continue;
    }
    auto filtered = FilterComponents(hw.first, hw.second, search_lower_, filter_level_);
    if (filtered.empty()) {
      continue;
    }
    rows.push_back({hw.first, std::move(filtered)});
  }

  if (rows.empty()) {
    tree_->hide();
    empty_label_->show();
    if (snapshot_.hardware.empty()) {
      empty_label_->setText(has_filter ? tr("No matching diagnostics") : tr("No diagnostic data"));
    } else {
      empty_label_->setText(has_filter ? tr("No matching diagnostics") : tr("No diagnostic data"));
    }
    filter_hint_->setVisible(has_filter);
    if (has_filter) {
      filter_hint_->setText(tr("No results with the current filter. Clear the filter or adjust the criteria."));
    }
    return;
  }

  tree_->show();
  empty_label_->hide();
  filter_hint_->setVisible(has_filter);
  if (has_filter) {
    filter_hint_->setText(tr("Showing %1 hardware group(s) (filter active)")
                              .arg(static_cast<int>(rows.size())));
  }

  for (const auto &entry : rows) {
    const std::string &hid = entry.first;
    const auto &filtered = entry.second;
    int max_lv = MaxLevelInMap(filtered);
    QString display_hid = hid == "unknown_hardware" ? tr("Unknown hardware") : QString::fromStdString(hid);
    auto *hw_item = new QTreeWidgetItem(tree_);
    QFont f = hw_item->font(0);
    f.setBold(true);
    hw_item->setFont(0, f);
    hw_item->setText(0, display_hid);
    hw_item->setForeground(1, LevelColor(max_lv));
    hw_item->setText(1, tr("Status: %1 | Components: %2 | Latest: %3")
                            .arg(LevelDisplayName(max_lv))
                            .arg(static_cast<int>(filtered.size()))
                            .arg(FormatTimeMs(LatestUpdateMs(filtered))));

    for (const auto &ce : filtered) {
      const std::string &comp_name = ce.first;
      const basic::DiagnosticComponentState &st = ce.second;
      auto *comp_item = new QTreeWidgetItem(hw_item);
      comp_item->setText(0, QString::fromStdString(comp_name));
      comp_item->setForeground(1, LevelColor(st.level));
      QString msg = QString::fromStdString(st.message);
      if (msg == QStringLiteral("data_stale")) {
        msg = tr("Data stale");
      }
      comp_item->setText(1, tr("Status: %1 | %2 | Updated: %3")
                                 .arg(LevelDisplayName(st.level))
                                 .arg(msg)
                                 .arg(FormatTimeMs(st.last_update_ms)));

      if (!st.key_values.empty()) {
        for (const auto &kv : st.key_values) {
          auto *kv_item = new QTreeWidgetItem(comp_item);
          kv_item->setText(0, QString::fromStdString(kv.first));
          kv_item->setText(1, QString::fromStdString(kv.second));
        }
      } else {
        auto *empty_item = new QTreeWidgetItem(comp_item);
        empty_item->setText(0, tr("(No key-value details)"));
        empty_item->setText(1, FormatTimeMs(st.last_update_ms));
      }
    }
  }
  tree_->expandToDepth(0);
}

