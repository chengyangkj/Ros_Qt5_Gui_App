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
#include "rviz/image/ros_image_texture.h"
#include <rviz/tool_manager.h>
#include <rviz/properties/property_tree_model.h>
#include <QThread>
#include <QDebug>
#include <QException>
#include <QAbstractItemModel>

#define RVIZ_DISPLAY_AXES               "rviz/Axes"
#define RVIZ_DISPLAY_EFFORT             "rviz/Errort"
#define RVIZ_DISPLAY_CAMERA             "rviz/Camera"
#define RVIZ_DISPLAY_GRID               "rviz/Grid"
#define RVIZ_DISPLAY_GRIDCELLS          "rviz/GridCells"
#define RVIZ_DISPLAY_IMAGE              "rviz/Image"
#define RVIZ_DISPLAY_INTERATIVEMARKER   "rviz/InteractiveMarker"
#define RVIZ_DISPLAY_LASERSCAN          "rviz/LaserScan"
#define RVIZ_DISPLAY_MAP                "rviz/Map"
#define RVIZ_DISPLAY_MARKERS            "rviz/Markers"
#define RVIZ_DISPLAY_PATH               "rviz/Path"
#define RVIZ_DISPLAY_POINT              "rviz/Point"
#define RVIZ_DISPLAY_POSE               "rviz/Pose"
#define RVIZ_DISPLAY_POSEARRAY          "rviz/PoseArray"
#define RVIZ_DISPLAY_POLYGON            "rviz/Polygon"
#define RVIZ_DISPLAY_ODOMETRY           "rviz/Odometry"
#define RVIZ_DISPLAY_RANGE              "rviz/Range"
#define RVIZ_DISPLAY_ROBOTMODEL         "rviz/RobotModel"
#define RVIZ_DISPLAY_TF                 "rviz/TF"
#define RVIZ_DISPLAY_WRENCH             "rviz/Wrench"
#define RVIZ_DISPLAY_OCULUS             "rviz/Oculus"

class QRviz:public QThread
{
    Q_OBJECT
public:
    QRviz(QVBoxLayout *layout,QString node_name);
    
    QList<rviz::Display *> rvizDisplays_;
    
    void DisplayInit(QString ClassID, bool enabled, QMap<QString, QVariant> namevalue);
    void DisplayInit(QString ClassID, QString name, bool enabled, QMap<QString, QVariant> namevalue);
    
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
    void Set_MoveCamera();
    void Set_Select();
    //发布goal话题的坐标
    void Send_Goal_topic();
    //显示robotmodel
    void Display_RobotModel(bool enable);
    
    void GetDisplayTreeModel();
    
signals:
    void ReturnModelSignal(QAbstractItemModel *model);
    
private:
    int GetDisplayNum(QString ClassID);
    int GetDisplayNum(QString ClassID, QString name);
    int GetDisplayNumName(QString name);
    
    //rviz显示容器
    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;
    rviz::Display* grid_=nullptr ;

    //显示tf坐标变换
    rviz::Display* TF_=nullptr ;
    rviz::Display* map_=nullptr ;
    rviz::Display* laser_=nullptr ;
    rviz::Display* Navigate_localmap=nullptr;
    rviz::Display* Navigate_localplanner=nullptr;
    rviz::Display* Navigate_globalmap=nullptr;
    rviz::Display* Navigate_globalplanner=nullptr;
    rviz::Display* Navigate_amcl=nullptr;
    rviz::Display* RobotModel_ = nullptr;



    //rviz工具
    rviz::Tool *current_tool;
    //rviz工具控制器
    rviz::ToolManager *tool_manager_;
    QVBoxLayout *layout;
    QString nodename;
    
    QMap<QString, QVariant> nullmap;
private slots:
    void addTool( rviz::Tool* );

 //   rviz::VisualizationManager *manager_=NULL;
//    rviz::RenderPanel *render_panel_;

};

#endif // QRVIZ_H
