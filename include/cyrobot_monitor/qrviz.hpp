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
#include <rviz/tool.h>
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
    //显示激光雷达点云
    void Display_LaserScan(bool enable,QString topic);
    void Set_Pos();
    void Set_Goal();
private:
    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;
    rviz::Display* grid_=NULL ;
    rviz::Display* map_=NULL ;
    rviz::Display* laser_=NULL ;
    rviz::Tool *current_tool;
    rviz::ToolManager *tool_manager_;
    QVBoxLayout *layout;
    QString nodename;
private slots:
    void addTool( rviz::Tool* );

 //   rviz::VisualizationManager *manager_=NULL;
//    rviz::RenderPanel *render_panel_;

};

#endif // QRVIZ_H
