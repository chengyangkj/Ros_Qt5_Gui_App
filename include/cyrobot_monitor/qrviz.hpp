#ifndef QRVIZ_H
#define QRVIZ_H

#include <QVBoxLayout>
#include <QThread>
#include <QDebug>
#include <QException>
#include <QAbstractItemModel>

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
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>
#include <rviz/display_group.h>

#define RVIZ_DISPLAY_AXES               "rviz/Axes"
#define RVIZ_DISPLAY_CAMERA             "rviz/Camera"
#define RVIZ_DISPLAY_DEPTHCLOUD         "rviz/DepthCloud"
#define RVIZ_DISPLAY_EFFORT             "rviz/Errort"
#define RVIZ_DISPLAY_FLUIDPRESSURE      "rviz/FluidPressure"
#define RVIZ_DISPLAY_GRID               "rviz/Grid"
#define RVIZ_DISPLAY_GRIDCELLS          "rviz/GridCells"
#define RVIZ_DISPLAY_GROUP              "rviz/Group"
#define RVIZ_DISPLAY_ILLUMINANCE        "rviz/Illuminance"
#define RVIZ_DISPLAY_IMAGE              "rviz/Image"
#define RVIZ_DISPLAY_INTERATIVEMARKER   "rviz/InteractiveMarker"
#define RVIZ_DISPLAY_LASERSCAN          "rviz/LaserScan"
#define RVIZ_DISPLAY_MAP                "rviz/Map"
#define RVIZ_DISPLAY_MARKER             "rviz/Marker"
#define RVIZ_DISPLAY_MARKERARRAY        "rviz/MarkerArray"
#define RVIZ_DISPLAY_ODOMETRY           "rviz/Odometry"
#define RVIZ_DISPLAY_PATH               "rviz/Path"
#define RVIZ_DISPLAY_POINTCLOUD         "rviz/PointCloud"
#define RVIZ_DISPLAY_POINTCLOUD2        "rviz/PointCloud2"
#define RVIZ_DISPLAY_POINTSTAMPED       "rviz/PointStamped"
#define RVIZ_DISPLAY_POLYGON            "rviz/Polygon"
#define RVIZ_DISPLAY_POSE               "rviz/Pose"
#define RVIZ_DISPLAY_POSEARRAY          "rviz/PoseArray"
#define RVIZ_DISPLAY_POSEWITHCOVARIANCE "rviz/PoseWithCovariance"
#define RVIZ_DISPLAY_RANGE              "rviz/Range"
#define RVIZ_DISPLAY_RELATIVEHUMIDITY   "rviz/RelativeHumidity"
#define RVIZ_DISPLAY_ROBOTMODEL         "rviz/RobotModel"
#define RVIZ_DISPLAY_TF                 "rviz/TF"
#define RVIZ_DISPLAY_TEMPERATURE        "rviz/Temperature"
#define RVIZ_DISPLAY_WRENCHSTAMPED      "rviz/WrenchStamped"

class QRviz:public QThread
{
    Q_OBJECT
public:
    QRviz(QVBoxLayout *layout,QString node_name);
    ~QRviz();
    
//    QList<rviz::Display *> rvizDisplays_;
    
    void DisplayInit(QString ClassID, bool enabled, QMap<QString, QVariant> namevalue);
    void DisplayInit(QString ClassID, QString name, bool enabled, QMap<QString, QVariant> namevalue);
    
    void RemoveDisplay(QString name);
    void RemoveDisplay(QString ClassID, QString name);
    
    void RenameDisplay(QString oldname, QString newname);
    
    void OutDisplaySet(QString path);
    void ReadDisplaySet(QString path);
    
    void run();
    void createDisplay(QString display_name,QString topic_name);
    //设置全局显示属性
    void SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate);

    void Set_Pos();
    void Set_Goal();
    void Set_MoveCamera();
    void Set_Select();
    //发布goal话题的坐标
    void Send_Goal_topic();
    
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

    rviz::DisplayGroup *display_group_;

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
