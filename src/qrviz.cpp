#include "../include/cyrobot_monitor/qrviz.hpp"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{
    int argc;
    char **argv;
    this->layout=layout;
    this->nodename=node_name;
    ros::init(argc,argv,"QRviz",ros::init_options::AnonymousName);
    //创建rviz容器
    render_panel_=new rviz::RenderPanel;
    //向layout添加widget
    layout->addWidget(render_panel_);
    //初始化rviz控制对象
    manager_=new rviz::VisualizationManager(render_panel_);
   //初始化camera 这行代码实现放大 缩小 平移等操作
    render_panel_->initialize(manager_->getSceneManager(),manager_);



//       //工具管理
//       rviz::ToolManager* tool_man;
//       connect( tool_man, SIGNAL( toolAdded( Tool* )), this, SLOT( addTool( Tool* )));
//       connect( tool_man, SIGNAL( toolRemoved( Tool* )), this, SLOT( removeTool( Tool* )));
//       connect( tool_man, SIGNAL( toolRefreshed( Tool* )), this, SLOT( refreshTool( Tool* )));
//       connect( tool_man, SIGNAL( toolChanged( Tool* )), this, SLOT( indicateToolIsCurrent( Tool* )));
    manager_->initialize();
    manager_->removeAllDisplays();

//    rviz::Display* grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
//    ROS_ASSERT( grid_ != NULL );
//    // Configure the GridDisplay the way we like it.
//    grid_->subProp( "Line Style" )->setValue("Billboards");
//    grid_->subProp( "Color" )->setValue(QColor(125,125,125));
//    manager_->startUpdate();



}
//显示grid
void QRviz::Display_Grid(bool enable,QColor color)
{
    if(grid_==NULL)
    {
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);

    }
    else{
        grid_->subProp( "Color" )->setValue(color);
    }
    grid_->setEnabled(enable);
    manager_->startUpdate();
}
//显示map
void QRviz::Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme)
{
    if(!enable&&map_)
    {
        map_->setEnabled(false);
        return ;
    }
    if(map_==NULL)
    {
        qDebug()<<"map ok";
        map_=manager_->createDisplay("rviz/Map","QMap",true);

        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);
    }
    map_->setEnabled(enable);
    manager_->startUpdate();
}
void QRviz::createDisplay(QString display_name,QString topic_name)
{


}
void QRviz::run()
{

}
