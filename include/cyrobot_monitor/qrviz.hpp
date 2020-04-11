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
    //显示导航相关控件
    void Display_Navigate(bool enable,QString Global_topic,QString Global_planner,QString Local_topic,QString Local_planner);
    //显示tf坐标变换
    void Display_TF(bool enable);
    void Set_Pos();
    void Set_Goal();
    //发布goal话题的坐标
    void Send_Goal_topic();
private:
    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;
    rviz::Display* grid_=NULL ;
    //显示tf坐标变换
    rviz::Display* TF_=NULL ;
    rviz::Display* map_=NULL ;
    rviz::Display* laser_=NULL ;
    rviz::Display *Navigate_localmap=NULL;
    rviz::Display *Navigate_localplanner=NULL;
    rviz::Display *Navigate_globalmap=NULL;
    rviz::Display *Navigate_globalplanner=NULL;
    rviz::Display *Navigate_amcl=NULL;
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
