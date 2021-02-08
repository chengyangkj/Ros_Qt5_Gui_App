/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cyrobot_monitor/qnode.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
//    读取topic的设置
    QSettings topic_setting("topic_setting","cyrobot_monitor");
    odom_topic= topic_setting.value("topic_odom","odom").toString();
    power_topic=topic_setting.value("topic_power","power").toString();
    laser_topic=topic_setting.value("topic_laser","scan").toString();
    pose_topic=topic_setting.value("topic_amcl","amcl_pose").toString();
    power_min=topic_setting.value("power_min","10").toString();
    power_max=topic_setting.value("power_max","12").toString();

    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cyrobot_monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  SubAndPubTopic();
	start();
	return true;
}

//初始化的函数*********************************
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cyrobot_monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  SubAndPubTopic();
	start();
	return true;
}
//创建订阅者与发布者
void QNode::SubAndPubTopic(){
  ros::NodeHandle n;
   // Add your ros communications here.

   //创建速度话题的订阅者
   cmdVel_sub =n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
   power_sub=n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
   //地图订阅
   map_sub = n.subscribe("map",1000,&QNode::mapCallback,this);
   //机器人位置话题
  pos_sub=n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
   //导航目标点发送话题
   goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
   //速度控制话题
   cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
   //激光雷达点云话题订阅
   m_laserSub=n.subscribe(laser_topic.toStdString(),1000,&QNode::laserScanCallback,this);
   //m_rosTimer=n.createTimer(ros::Duration(1.0),boost::bind(&QNode::transformPoint,boost::ref(m_tfListener)));
   //全局规划Path
   m_plannerPathSub=n.subscribe("/move_base/NavfnROS/plan",1000,&QNode::plannerPathCallback,this);
}

void QNode::transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;
    for(auto topic:topic_list)
    {
        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));
    }
    return res;
}
//planner的路径话题回调
void QNode::plannerPathCallback(nav_msgs::Path::ConstPtr path){
     plannerPoints.clear();
     for(int i=0;i<path->poses.size();i++){
       QPointF roboPos;
       roboPos.setX((path->poses[i].pose.position.x-m_mapOriginX)/m_mapResolution);
       roboPos.setY((path->poses[i].pose.position.y-m_mapOriginY)/m_mapResolution);
       plannerPoints.append(roboPos);
     }
     emit plannerPath(plannerPoints);
}

//激光雷达点云话题回调
void QNode::laserScanCallback(sensor_msgs::LaserScanConstPtr laser_msg){
  //获取tf变换 机器人坐标系变换到map坐标系
  tf::TransformListener listener;
  tf::StampedTransform transform;
  static float qx=0,qy=0;
  geometry_msgs::PointStamped laser_point;
  geometry_msgs::PointStamped map_point;
  laser_point.header.frame_id = laser_msg->header.frame_id;
  std::vector<float> ranges = laser_msg->ranges;
  laserPoints.clear();
  //转换到二维XY平面坐标系下;
  for(int i=0; i< ranges.size(); i++)
  {
    //scan_laser坐标系下
    double angle = laser_msg->angle_min + i * laser_msg->angle_increment;
    double X = ranges[i] * cos(angle);
    double Y = ranges[i] * sin(angle);
    laser_point.point.x=X;
    laser_point.point.y=Y;
    laser_point.point.z = 0.0;
    //change to map frame
    try{
      listener.waitForTransform("odom",laser_point.header.frame_id,ros::Time(0),ros::Duration(0.4));
      listener.transformPoint("odom", laser_point, map_point);
      listener.waitForTransform("map","odom",ros::Time(0),ros::Duration(0.4));
      listener.lookupTransform("map","odom",ros::Time(0), transform);
      map_point.point.x += transform.getOrigin().x();
      map_point.point.y += transform.getOrigin().y();
    }
    catch(tf::TransformException& ex){
      //ROS_ERROR("Received an exception trying to transform  %s", ex.what());
    }
    QPointF roboPos;
    roboPos.setX((map_point.point.x-m_mapOriginX)/m_mapResolution);
    roboPos.setY((map_point.point.y-m_mapOriginY)/m_mapResolution);
    laserPoints.append(roboPos);
  }
  updateLaserScan(laserPoints);
}

//机器人位置话题的回调函数
void QNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
      QPointF roboPos;
      roboPos.setX((pos.pose.pose.position.x-m_mapOriginX)/m_mapResolution);
      roboPos.setY((pos.pose.pose.position.y-m_mapOriginY)/m_mapResolution);
      //qDebug()<<"callback pose:"<<roboPos;
      //yaw
      tf::Quaternion quat;
      tf::quaternionMsgToTF(pos.pose.pose.orientation, quat);
      double roll, pitch, yaw;//定义存储r\p\y的容器
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
      emit updateRoboPose(roboPos,yaw);
}
void QNode::powerCallback(const std_msgs::Float32 &message_holder)
{
    emit power(message_holder.data);
}
void QNode::myCallback(const std_msgs::Float64 &message_holder)
{
    qDebug()<<message_holder.data<<endl;
}
//发布导航目标点信息
void QNode::set_goal(QString frame,double x,double y,double z,double w)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id=frame.toStdString();
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.position.z=0;
    goal.pose.orientation.z=z;
    goal.pose.orientation.w=w;
    goal_pub.publish(goal);
    ros::spinOnce();
}
//地图信息订阅回调函数
void QNode::mapCallback(nav_msgs::OccupancyGrid::ConstPtr map){
      mapWidth=map->info.width;
      mapHeight=map->info.height;
      m_mapOriginX=map->info.origin.position.x;
      m_mapOriginY=map->info.origin.position.y;
      m_mapResolution=map->info.resolution;
      int row, col, value;
      cv::Mat image(map->info.width, map->info.height, CV_8UC1);
      for(int array_index=0; array_index < map->data.size(); array_index++) {
          //计算当前所在行
          row = (int)array_index/image.cols;
          //计算当前所在列
          col = array_index%image.cols;
          //获取当前位置的像素值
          int curr_data=map->data[array_index];
          //计算值
          if ( curr_data== -1){
              value = 125;    // grey
          } else if (curr_data == 100) {
              value = 0;      // black
          } else if (curr_data == 0) {
              value = 255;    // white
          } else {
              ROS_WARN("Unsupported value in Occupancy Grid");
              value = 125;
          }
          image.at<uchar>(row, col) = (uchar)value;
      }
      QImage imageMap=Mat2QImage(image);
      imageMap.save("/home/chengyangkj/map.png","png",100);
      QSizeF size(mapWidth,mapHeight);
      emit updateMap(imageMap);
}

//速度回调函数
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.linear.y);
}
void QNode::run() {
        ros::Rate loop_rate(30);
        //当当前节点没有关闭时
        while ( ros::ok() ) {
            //调用消息处理回调函数
            ros::spinOnce();

            loop_rate.sleep();
        }
        //如果当前节点关闭
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)


}
//发布机器人速度控制
 void QNode::move_base(char k,float speed_linear,float speed_trun)
 {
     std::map<char, std::vector<float>> moveBindings
     {
       {'i', {1, 0, 0, 0}},
       {'o', {1, 0, 0, -1}},
       {'j', {0, 0, 0, 1}},
       {'l', {0, 0, 0, -1}},
       {'u', {1, 0, 0, 1}},
       {',', {-1, 0, 0, 0}},
       {'.', {-1, 0, 0, 1}},
       {'m', {-1, 0, 0, -1}},
       {'O', {1, -1, 0, 0}},
       {'I', {1, 0, 0, 0}},
       {'J', {0, 1, 0, 0}},
       {'L', {0, -1, 0, 0}},
       {'U', {1, 1, 0, 0}},
       {'<', {-1, 0, 0, 0}},
       {'>', {-1, -1, 0, 0}},
       {'M', {-1, 1, 0, 0}},
       {'t', {0, 0, 1, 0}},
       {'b', {0, 0, -1, 0}},
       {'k', {0, 0, 0, 0}},
       {'K', {0, 0, 0, 0}}
     };
     char key=k;
     //计算是往哪个方向
     float x = moveBindings[key][0];
     float y = moveBindings[key][1];
     float z = moveBindings[key][2];
     float th = moveBindings[key][3];
     //计算线速度和角速度
     float speed = speed_linear;
     float turn = speed_trun;
     // Update the Twist message
     geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    cmd_pub.publish(twist);
    ros::spinOnce();

 }
 //订阅图片话题，并在label上显示
 void QNode::Sub_Image(QString topic,int frame_id)
 {
      ros::NodeHandle n;
      image_transport::ImageTransport it_(n);
       image_sub0=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback0,this);
       ros::spinOnce();
 }
 //图元坐标系转换为map坐标系
QPointF QNode::transScenePoint2Map(QPointF pos){
  QPointF roboPos;
  roboPos.setX(pos.x()*m_mapResolution+m_mapOriginX);
  roboPos.setY(pos.y()*m_mapResolution+m_mapOriginY);
  return roboPos;
}
//map坐标系转换为图元坐标系
QPointF QNode::transMapPoint2Scene(QPointF pos){
 QPointF roboPos;
 roboPos.setX((pos.x()-m_mapOriginX)/m_mapResolution);
 roboPos.setY((pos.y()-m_mapOriginY)/m_mapResolution);
 return roboPos;
}
 //图像话题的回调函数
 void QNode::imageCallback0(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;

     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(0,im);
       }
       catch (cv_bridge::Exception& e)
       {

         log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
         return;
       }
 }

 QImage QNode::Mat2QImage(cv::Mat const& src)
 {
   QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

   const float scale = 255.0;

   if (src.depth() == CV_8U) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = src.at<quint8>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
         }
       }
     }
   } else if (src.depth() == CV_32F) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = scale * src.at<float>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
         }
       }
     }
   }

   return dest;
 }
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace cyrobot_monitor
