#ifndef QRVIZ_H
#define QRVIZ_H

#include <QVBoxLayout>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool_manager.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <QThread>
#include <QDebug>
class QRviz:public QThread
{
    Q_OBJECT
public:
    QRviz(QVBoxLayout *layout,QString node_name);
    void run();
    void createDisplay(QString display_name,QString topic_name);
    void Display_Grid(bool enable,QColor color=QColor(125,125,125));
    void Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme);

private:
    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;
    rviz::Display* grid_=NULL ;
    rviz::Display* map_=NULL ;
    QVBoxLayout *layout;
    QString nodename;

 //   rviz::VisualizationManager *manager_=NULL;
//    rviz::RenderPanel *render_panel_;

};

#endif // QRVIZ_H
