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
    odom_topic= topic_setting.value("topic_odom","raw_odom").toString();
    power_topic=topic_setting.value("topic_power","power").toString();
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
    ros::NodeHandle n;
	// Add your ros communications here.

    //创建速度话题的订阅者
    cmdVel_sub =n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub=n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub=n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
     //导航目标点发送话题
     goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
     //速度控制话题
     cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
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
    ros::NodeHandle n;
    //创建速度话题的订阅者
    cmdVel_sub =n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub=n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub=n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
    //导航目标点发送话题
    goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    //速度控制话题
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	start();
	return true;
}

//机器人位置话题的回调函数
void QNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
    emit position(pos.header.frame_id.data(), pos.pose.pose.position.x,pos.pose.pose.position.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
//    qDebug()<<<<" "<<pos.pose.pose.position.y;
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
//速度回调函数
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.linear.y);
}
void QNode::run() {
        int count=0;
        ros::Rate loop_rate(1);
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
 void QNode::Sub_Image(QString topic,int frame_id,QString format)
 {
      ros::NodeHandle n;
      image_transport::ImageTransport it_(n);
     switch (frame_id) {
         case 0:
            video0_format=format;
            image_sub0=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback0,this);
         break;
         case 1:
             video1_format=format;
             image_sub1=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback1,this);
          break;
         case 2:
             video2_format=format;
             image_sub2=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback2,this);
          break;
         case 3:
             video3_format=format;
             image_sub3=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback3,this);
          break;
     }
     ros::spinOnce();
 }

 //图像话题的回调函数
 void QNode::imageCallback0(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;

     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, video0_format.toStdString());
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(0,im);
       }
       catch (cv_bridge::Exception& e)
       {

         log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg,video1_format.toStdString());
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(1,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame1 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, video2_format.toStdString());
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(2,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame2 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, video3_format.toStdString());
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(3,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame3 exception: "+QString(e.what())).toStdString());
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
