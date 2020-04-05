#include "../include/cyrobot_monitor/qrviz.h"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{
    this->layout=layout;
    this->nodename=node_name;
    start();

//        //创建rviz地图节点
//        rviz::Display *map_=manager_->createDisplay("rviz/Map","adjustable map",true);
//        ROS_ASSERT(map_!=NULL);
//        //订阅地图话题
//        map_->subProp("Topic")->setValue("/map");
//        manager_->startUpdate();
//        //机器人模型
//        rviz::Display *robot_=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
//         ROS_ASSERT(robot_!=NULL);
//        robot_->subProp("Robot Description")->setValue("robot_description");

//        //激光雷达话题
//        rviz::Display *laser_=manager_->createDisplay("rviz/LaserScan","adjustable scan",true);
//        ROS_ASSERT(laser_!=NULL);
//        laser_->subProp("Topic")->setValue("/scan");
//        laser_->subProp("Size (m)")->setValue("0.1");

}
void QRviz::createDisplay(QString display_name,QString topic_name)
{


}
void QRviz::run()
{

}
