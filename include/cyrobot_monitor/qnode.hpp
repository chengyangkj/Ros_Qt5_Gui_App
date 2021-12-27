/**
 * @file /include/cyrobot_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cyrobot_monitor_QNODE_HPP_
#define cyrobot_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>  //cv_bridge
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>  //image_transport
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>  //图像编码格式
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include <QDebug>
#include <QImage>
#include <QLabel>
#include <QSettings>
#include <QStringListModel>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <string>

#include "robomap.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void move_base(char k, float speed_linear, float speed_trun);
  void set_goal(QString frame, double x, double y, double z, double w);
  void Sub_Image(QString topic, int frame_id);
  void pub_imageMap(QImage map);
  double getRealTheta(QPointF start, QPointF end);
  QPointF transScenePoint2Map(QPointF pos);
  QPointF transMapPoint2Scene(QPointF pos);
  QMap<QString, QString> get_topic_list();
  int mapWidth{0};
  int mapHeight{0};
  void run();

  /*********************
  ** Logging
  **********************/
  enum LogLevel { Debug, Info, Warn, Error, Fatal };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

 public slots:
  void slot_pub2DPos(algo::RobotPose pose);
  void slot_pub2DGoal(algo::RobotPose pose);
 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void speed_x(double x);
  void speed_y(double y);
  void batteryState(sensor_msgs::BatteryState);
  void Master_shutdown();
  void Show_image(int, QImage);
  void updateRoboPose(algo::RobotPose pos);
  void updateMap(QImage map);
  void plannerPath(QPolygonF path);
  void updateLaserScan(QPolygonF points);
  void updateRobotStatus(algo::RobotStatus status);

 private:
  int init_argc;
  char **init_argv;
  ros::Publisher chatter_publisher;
  ros::Subscriber cmdVel_sub;
  ros::Subscriber chatter_subscriber;
  ros::Subscriber pos_sub;
  ros::Subscriber m_laserSub;
  ros::Subscriber battery_sub;
  ros::Subscriber m_plannerPathSub;
  ros::Subscriber m_compressedImgSub0;
  ros::Subscriber m_compressedImgSub1;
  ros::Publisher goal_pub;
  ros::Publisher cmd_pub;
  ros::Publisher m_initialposePub;
  image_transport::Publisher m_imageMapPub;
  MoveBaseClient *movebase_client;
  QStringListModel logging_model;
  QString show_mode = "control";
  //图像订阅
  image_transport::Subscriber image_sub0;
  //地图订阅
  ros::Subscriber map_sub;
  //图像format
  QString video0_format;
  QString odom_topic;
  QString batteryState_topic;
  QString pose_topic;
  QString laser_topic;
  QString map_topic;
  QString initPose_topic;
  QString naviGoal_topic;
  std::string path_topic;
  QPolygon mapPonits;
  QPolygonF plannerPoints;
  QPolygonF laserPoints;
  int m_threadNum = 4;
  int m_frameRate = 40;
  //地图 0 0点坐标对应世界坐标系的坐标
  float m_mapOriginX;
  float m_mapOriginY;
  //地图坐标系中心点坐标
  QPointF m_mapCenterPoint;
  //图元坐标系中心点坐标
  QPointF m_sceneCenterPoint;
  //地图一个像素对应真实世界的距离
  float m_mapResolution;
  //地图是否被初始化
  bool m_bMapIsInit = false;
  // tf::TransformListener m_tfListener(ros::Duration(10));
  // ros::Timer m_rosTimer;
  QImage Mat2QImage(cv::Mat const &src);
  cv::Mat QImage2Mat(QImage &image);
  cv::Mat RotaMap(cv::Mat const &map);
  tf::TransformListener *m_robotPoselistener;
  tf::TransformListener *m_Laserlistener;
  std::string base_frame, laser_frame, map_frame;

 private:
  void speedCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void batteryCallback(const sensor_msgs::BatteryState &message);
  void imageCallback0(const sensor_msgs::CompressedImageConstPtr &msg);
  void imageCallback1(const sensor_msgs::CompressedImageConstPtr &msg);
  void myCallback(const std_msgs::Float64 &message_holder);
  void mapCallback(nav_msgs::OccupancyGrid::ConstPtr map);
  void laserScanCallback(sensor_msgs::LaserScanConstPtr scan);
  void plannerPathCallback(nav_msgs::Path::ConstPtr path);
  void SubAndPubTopic();
  void updateRobotPose();
};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
