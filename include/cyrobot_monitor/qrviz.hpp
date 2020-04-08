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
    //显示Grid
    void Display_Grid(bool enable,QString Reference_frame,int Plan_Cell_count,QColor color=QColor(125,125,125));
    //显示map
    void Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme);
    //设置全局显示属性
    void SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate);

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
