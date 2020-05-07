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
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
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
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void move_base(char k,float speed_linear,float speed_trun);
    void set_goal(QString frame,double x,double y,double z,double w);
    void Sub_Image(QString topic,int frame_id,QString format);

	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
             Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void speed_x(double x);
    void speed_y(double y);
    void power(float p);
    void Master_shutdown();
    void Show_image(int,QImage);
    void position(QString frame,double x,double y,double z,double w);
private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber power_sub;
    ros::Publisher goal_pub;
    ros::Publisher cmd_pub;
    QStringListModel logging_model;
    //图像订阅
    image_transport::Subscriber image_sub0;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;
    QString odom_topic;
    QString power_topic;
    QString pose_topic;
    QString power_max;
    QString power_min;
    QImage Mat2QImage(cv::Mat const& src);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void powerCallback(const std_msgs::Float32& message_holder);
    void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
    void myCallback(const std_msgs::Float64& message_holder);
};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
