#ifndef QRVIZ_WIDGET_H
#define QRVIZ_WIDGET_H

#include <QWidget>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool_manager.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
namespace Ui {
class QRviz_widget;
}

class QRviz_widget : public QWidget
{
    Q_OBJECT

public:
    explicit QRviz_widget(QWidget *parent = 0);
    ~QRviz_widget();

private:
    Ui::QRviz_widget *ui;
};

#endif // QRVIZ_WIDGET_H
