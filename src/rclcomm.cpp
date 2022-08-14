#include "rclcomm.h"
rclcomm::rclcomm()
{
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);
    m_executor =new rclcpp::executors::MultiThreadedExecutor;
    node=rclcpp::Node::make_shared("ros2_qt5_gui_app");
    m_executor->add_node(node);
    callback_group_laser =node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_other =node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub1_obt = rclcpp::SubscriptionOptions();
    sub1_obt.callback_group=callback_group_other;
    auto sub_laser_obt = rclcpp::SubscriptionOptions();
    sub_laser_obt.callback_group=callback_group_laser;

    m_map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),std::bind(&rclcomm::map_callback,this,std::placeholders::_1),sub1_obt);
    m_laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan",20,std::bind(&rclcomm::laser_callback,this,std::placeholders::_1),sub_laser_obt);
    m_path_sub =node->create_subscription<nav_msgs::msg::Path>("/plan",20,std::bind(&rclcomm::path_callback,this,std::placeholders::_1),sub1_obt);
    m_tf_buffer=std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener=std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}
void rclcomm::getRobotPose(){
    try {
      geometry_msgs::msg::TransformStamped transform =  m_tf_buffer->lookupTransform("map","base_link",tf2::TimePointZero);
      geometry_msgs::msg::Quaternion msg_quat = transform.transform.rotation;
      //转换类型
      tf2::Quaternion q;
      tf2::fromMsg(msg_quat,q);
      tf2::Matrix3x3 mat(q);
      double roll,pitch,yaw;
      mat.getRPY(roll,pitch,yaw);
      //x y
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      QPointF trans_pose = transWordPoint2Scene(QPointF(x,y));
      m_currPose.x=trans_pose.x();
      m_currPose.y=trans_pose.y();
      m_currPose.theta=yaw;
      emit emitUpdateRobotPose(m_currPose);
    } catch (tf2::TransformException &ex) {
        qDebug()<<"robot pose transform error:"<<ex.what();
    }
}
void rclcomm::run(){
    std_msgs::msg::Int32 pub_msg;
    pub_msg.data=0;
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
     pub_msg.data++;
     m_executor->spin_some();
     getRobotPose();
    loop_rate.sleep();
    }
    rclcpp::shutdown();
}
 QImage rclcomm::rotateMapWithY(QImage map){
     QImage res=map;
     for(int x=0;x<map.width();x++){
         for(int y=0;y<map.height();y++){
             res.setPixelColor(x,map.height()-y-1,map.pixel(x,y));
         }
     }
     return res;
 }
 QPointF rclcomm::transWordPoint2Scene(QPointF point){
     QPointF ret;
     ret.setX(m_wordOrigin.x()+ point.x()/m_resolution);
     ret.setY(m_wordOrigin.y()-point.y()/m_resolution);
     return ret;
 }
QPointF rclcomm::transScenePoint2Word(QPointF point){
 QPointF ret;
 ret.setX( (point.x()-m_wordOrigin.x())*m_resolution);
 ret.setY(-1*(point.y()-m_wordOrigin.y())*m_resolution);
 return ret;
}
void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
    qDebug()<<"path frame id:"<<QString::fromStdString(msg->header.frame_id);
    qDebug()<<"num:"<<msg->poses.size();
    QPolygonF emit_points;
    for(int i=0;i<msg->poses.size();i++){
        double x = msg->poses.at(i).pose.position.x;
        double y = msg->poses.at(i).pose.position.y;
        QPointF point ;
        point.setX(x);
        point.setY(y);
        point = transWordPoint2Scene(point);
        emit_points.push_back(point);
//        qDebug()<<"x:"<<x<<" y:"<<y<<" trans:"<<point.x()<<" "<<point.y();
    }
    emit emitUpdatePath(emit_points);
}
void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
//   qDebug()<<"订阅到激光话题";
    double angle_min=msg->angle_min;
    double angle_max=msg->angle_max;
    double angle_increment=msg->angle_increment;
    try {
//        geometry_msgs::msg::TransformStamped laser_transform =  m_tf_buffer->lookupTransform("map","base_scan",tf2::TimePointZero);
        geometry_msgs::msg::PointStamped point_map_frame;
        geometry_msgs::msg::PointStamped point_laser_frame;
        QPolygonF emit_points;
        for(int i=0;i<msg->ranges.size();i++){
             //计算当前偏移角度
            double angle = angle_min+i*angle_increment;

            double x=msg->ranges[i]*cos(angle);
            double y=msg->ranges[i]*sin(angle);
            point_laser_frame.point.x=x;
            point_laser_frame.point.y=y;
            point_laser_frame.header.frame_id = msg->header.frame_id;

//            tf2::doTransform(point_laser_frame,point_map_frame,laser_transform);
            m_tf_buffer->transform(point_laser_frame,point_map_frame,"map");
//            qDebug()<<"转换前:"<<x<<" "<<y<<" 转换后:"<<point_map_frame.point.x<<" "<<point_map_frame.point.y;
            QPointF laser_scene_point;
            laser_scene_point.setX(point_map_frame.point.x);
            laser_scene_point.setY(point_map_frame.point.y);
            laser_scene_point =transWordPoint2Scene(laser_scene_point);
            emit_points.push_back(laser_scene_point);
        }
        emit emitUpdateLaserPoint(emit_points);
    }  catch (tf2::TransformException &ex) {
        qDebug()<<"laser pose transform error:"<<ex.what();
    }

}
void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
   double origin_x = msg->info.origin.position.x;
   double origin_y = msg->info.origin.position.y;
   qDebug()<<"我收到map话题 origin_x:"<<origin_x<<" origin_y:"<<origin_y;
   int width =msg->info.width;
   int height =msg->info.height;
  m_resolution = msg->info.resolution;
   QImage map_image(width,height,QImage::Format_RGB32);
   for(int i=0;i<msg->data.size();i++){
      int x=i%width;
      int y=int(i/width);
      QColor color;
      if(msg->data[i]==100){
        color = Qt::black; //黑色
      }else if(msg->data[i]==0){
        color = Qt::white; //白色
      }else{
        color = Qt::gray;
      }
      map_image.setPixel(x,y,qRgb(color.red(),color.green(),color.blue()));
   }
//   map_image.save("/home/chengyangkj/map.png");
   QImage rotate_map=rotateMapWithY(map_image);
//   rotate_map.save("/home/chengyangkj/rotate_map.png");
   emit emitUpdateMap(rotate_map);
   //计算图元坐标系原点在世界坐标系下的坐标(翻转之后的栅格地图坐标原点在世界坐标系下的坐标)
   double trans_origin_x = origin_x;
   double trans_origin_y = origin_y+height*m_resolution;
   //世界坐标系原点在图元坐标系下的坐标
   m_wordOrigin.setX(fabs(trans_origin_x/m_resolution));
   m_wordOrigin.setY(fabs(trans_origin_y/m_resolution));
}
void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg){
//     qDebug()<<msg->data;
     emit emitTopicData("i am listen from topic:" +QString::fromStdString(std::to_string(msg->data)));
}
