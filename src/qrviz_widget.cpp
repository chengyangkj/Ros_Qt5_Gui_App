#include "../include/cyrobot_monitor/qrviz_widget.h"
#include "ui_qrviz_widget.h"

QRviz_widget::QRviz_widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QRviz_widget)
{
    ui->setupUi(this);
        int argc;
        char **argv;
        ros::init(argc,argv,"QRviz",ros::init_options::AnonymousName);
        //创建rviz容器
        rviz::RenderPanel *render_panel_=new rviz::RenderPanel;
    //    //设置鼠标形状
        render_panel_->setCursor(Qt::PointingHandCursor);

        //向layout添加widget
        ui->verticalLayout->addWidget(render_panel_);
        //初始化rviz控制对象
       rviz::VisualizationManager* manager_=new rviz::VisualizationManager(render_panel_);
           manager_->initialize();
           manager_->removeAllDisplays();

       rviz::Display* grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );

        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue( "Billboards" );

        grid_->subProp( "Color" )->setValue(QColor(125,125,125));
        manager_->startUpdate();
}

QRviz_widget::~QRviz_widget()
{
    delete ui;
}
