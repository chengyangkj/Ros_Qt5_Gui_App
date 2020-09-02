/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cyrobot_monitor/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    initUis();
    initData();
    //读取配置文件
    ReadSettings();
    setWindowIcon(QIcon(":/images/robot.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());

    /*********************
    ** 自动连接master
    **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
//    }
    //链接connect
    connections();

}

void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);
}

//订阅video话题
void MainWindow::initVideos()
{

   QSettings video_topic_setting("video_topic","cyrobot_monitor");
   QStringList names=video_topic_setting.value("names").toStringList();
   QStringList topics=video_topic_setting.value("topics").toStringList();
   if(names.size()==4)
   {
       ui.label_v_name0->setText(names[0]);
       ui.label_v_name1->setText(names[1]);
       ui.label_v_name2->setText(names[2]);
       ui.label_v_name3->setText(names[3]);
   }
   if(topics.size()==4)
   {
       if(topics[0]!="")
        qnode.Sub_Image(topics[0],0);
       if(topics[1]!="")
        qnode.Sub_Image(topics[1],1);
       if(topics[2]!="")
        qnode.Sub_Image(topics[2],2);
       if(topics[3]!="")
        qnode.Sub_Image(topics[3],3);

   }

   //链接槽函数
   connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));


}
void MainWindow::slot_show_image(int frame_id, QImage image)
{
    switch (frame_id)
    {
    case 0:
        ui.label_video0->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video0->width(),ui.label_video0->height()));
        break;
    case 1:
        ui.label_video1->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video1->width(),ui.label_video1->height()));
        break;
    case 2:
        ui.label_video2->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video2->width(),ui.label_video2->height()));
        break;
    case 3:
        ui.label_video3->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video3->width(),ui.label_video3->height()));
        break;
    }
}
//初始化UI
void MainWindow::initUis()
{
    ui.groupBox_3->setEnabled(false);
    m_DashBoard_x =new CCtrlDashBoard(ui.widget_speed_x);
    m_DashBoard_x->setGeometry(ui.widget_speed_x->rect());
    m_DashBoard_x->setValue(0);
    m_DashBoard_y =new CCtrlDashBoard(ui.widget_speed_y);
    m_DashBoard_y->setGeometry(ui.widget_speed_y->rect());
    m_DashBoard_y->setValue(0);

    ui.tab_manager->setCurrentIndex(0);
    ui.tabWidget->setCurrentIndex(0);
    
    ui.pushButton_remove_topic->setEnabled(false);
    ui.pushButton_rename_topic->setEnabled(false);

    //qucik treewidget
    ui.treeWidget_quick_cmd->setHeaderLabels(QStringList()<<"key"<<"values");
    ui.treeWidget_quick_cmd->setHeaderHidden(true);

}
void MainWindow::initRviz()
{
    ui.label_rvizShow->hide();
    map_rviz_=new QRviz(ui.verticalLayout_build_map,"qrviz");
    connect(map_rviz_, &QRviz::ReturnModelSignal, this, &MainWindow::RvizGetModel);
    map_rviz_->GetDisplayTreeModel();
    QMap<QString, QVariant> namevalue;
    namevalue.insert("Line Style", "Billboards");
    namevalue.insert("Color", QColor(160, 160, 160));
    namevalue.insert("Plane Cell Count", 10);
    map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);
}

void MainWindow::RvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;
    ui.treeView_rvizDisplayTree->setModel(model);
}

void MainWindow::connections()
{
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));
    //connect速度的信号
    connect(&qnode,SIGNAL(speed_x(double)),this,SLOT(slot_speed_x(double)));
    connect(&qnode,SIGNAL(speed_y(double)),this,SLOT(slot_speed_y(double)));
    //电源的信号
    connect(&qnode,SIGNAL(power(float)),this,SLOT(slot_power(float)));
    //机器人位置信号
    connect(&qnode,SIGNAL(position(QString,double,double,double,double)),this,SLOT(slot_position_change(QString,double,double,double,double)));
    //绑定快捷按钮相关函数
    connect(ui.quick_cmd_add_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_add()));
    connect(ui.quick_cmd_remove_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_remove()));
    //绑定slider的函数
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_raw_valueChanged(int)));
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_linear_valueChanged(int)));
    //设置界面
    connect(ui.action_2,SIGNAL(triggered(bool)),this,SLOT(slot_setting_frame()));
    //绑定速度控制按钮
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_back,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui.pushButton_backr,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    //设置2D Pose
    connect(ui.set_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Pos()));
    //设置2D goal
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Goal()));
    //设置MoveCamera
    connect(ui.move_camera_btn,SIGNAL(clicked()),this,SLOT(slot_move_camera_btn()));
    //设置Select
    connect(ui.set_select_btn,SIGNAL(clicked()),this,SLOT(slot_set_select()));
    //设置返航点
    connect(ui.set_return_btn,SIGNAL(clicked()),this,SLOT(slot_set_return_point()));
    //返航
    connect(ui.return_btn,SIGNAL(clicked()),this,SLOT(slot_return_point()));
    //左工具栏tab索引改变
    connect(ui.tab_manager,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_manage_currentChanged(int)));
    //右工具栏索引改变
    connect(ui.tabWidget,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_Widget_currentChanged(int)));
    //刷新话题列表
    connect(ui.refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList()));
    //添加rviz话题的按钮
    connect(ui.pushButton_add_topic,SIGNAL(clicked()),this,SLOT(slot_add_topic_btn()));
    //treewidget的值改变的槽函数

    
    //connect(ui.treeWidget_rviz,SIGNAL(itemChanged(QTreeWidgetItem*,int)),this,SLOT(slot_treewidget_item_value_change(QTreeWidgetItem*,int)));
}
//设置界面
void MainWindow::slot_setting_frame()
{
    if(set != nullptr)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else
    {
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    //绑定set确认按钮点击事件
}
//刷新当前坐标
void MainWindow::slot_position_change(QString frame,double x,double y,double z,double w)
{
    //更新ui显示
    ui.label_frame->setText(frame);
    ui.label_x->setText(QString::number(x));
    ui.label_y->setText(QString::number(y));
    ui.label_z->setText(QString::number(z));
    ui.label_w->setText(QString::number(w));
}
//刷新返航地点
void MainWindow::slot_set_return_point()
{
    //更新ui返航点显示
    ui.label_return_x->setText(ui.label_x->text());
    ui.label_return_y->setText(ui.label_y->text());
    ui.label_return_z->setText(ui.label_z->text());
    ui.label_return_w->setText(ui.label_w->text());
    //写入setting
    QSettings settings("return-position", "cyrobot_monitor");
    settings.setValue("x",ui.label_x->text());
    settings.setValue("y",ui.label_y->text());
    settings.setValue("z",ui.label_z->text());
    settings.setValue("w",ui.label_w->text());
    //发出声音提醒
    if(media_player!=nullptr)
     {
         delete media_player;
         media_player=nullptr;
     }
     media_player=new QSoundEffect;
     media_player->setSource(QUrl::fromLocalFile("://media/refresh_return.wav"));
     media_player->play();

}
//返航
void MainWindow::slot_return_point()
{
    qnode.set_goal(ui.label_frame->text(),ui.label_return_x->text().toDouble(),ui.label_return_y->text().toDouble(),ui.label_return_z->text().toDouble(),ui.label_return_w->text().toDouble());
    if(media_player!=nullptr)
       {
           delete media_player;
           media_player=nullptr;
       }
       media_player=new QSoundEffect;
       media_player->setSource(QUrl::fromLocalFile("://media/start_return.wav"));
       media_player->play();
}
//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz_->Set_Pos();
// ui.label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz_->Set_Goal();
//  ui.label_map_msg->setText("请在地图中选择机器人导航的目标位置");
}
void MainWindow::slot_move_camera_btn()
{
    map_rviz_->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz_->Set_Select();
}


//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);
    // 检查重名
    name = JudgeDisplayNewName(name);
    
    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    map_rviz_->DisplayInit(m_mapRvizDisplays[ClassID], name, true, namevalue);
//    if (ClassID == "Axes")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_AXES, name, true, namevalue);
//    }
//    else if(ClassID=="Map")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_MAP, name, true, namevalue);
//    }
//    else if (ClassID == "Grid")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, name, true, namevalue);
//    }
//    else if (ClassID=="LaserScan")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_LASERSCAN, name, true, namevalue);
//    }
//    else if (ClassID == "Camera")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_CAMERA, name, true, namevalue);
//    }
//    else if (ClassID == "Image")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_IMAGE, name, true, namevalue);
//    }
//    else if (ClassID == "RobotModel")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_ROBOTMODEL, name, true, namevalue);
//    }
//    else if(ClassID == "TF")
//    {
//        map_rviz_->DisplayInit(RVIZ_DISPLAY_TF, name, true, namevalue);
//    }
//    else// if (ClassID=="Navigate")
//    {
//        inform("当前不支持选择此项");
//    }
}

///
/// \brief 检查重名
/// \param name
/// \return 
///
QString MainWindow::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                //m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else {
                      name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}

//左工具栏索引改变
void MainWindow::slot_tab_manage_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:

        ui.tabWidget->setCurrentIndex(1);
        break;
    case 2:
        break;

    }
}
//右工具栏索引改变
void MainWindow::slot_tab_Widget_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:
        ui.tab_manager->setCurrentIndex(1);
        break;
    case 2:
        break;

    }
}
//速度控制相关按钮处理槽函数
void MainWindow::slot_cmd_control()
{

    QPushButton* btn=qobject_cast<QPushButton*>(sender());
    char key=btn->text().toStdString()[0];
    //速度
    float liner=ui.horizontalSlider_linear->value()*0.01f;
    float turn=ui.horizontalSlider_raw->value()*0.01f;
    bool is_all=ui.checkBox_use_all->isChecked();
    switch (key) {
        case 'u':
            qnode.move_base(is_all?'U':'u',liner,turn);
        break;
        case 'i':
            qnode.move_base(is_all?'I':'i',liner,turn);
        break;
        case 'o':
            qnode.move_base(is_all?'O':'o',liner,turn);
        break;
        case 'j':
            qnode.move_base(is_all?'J':'j',liner,turn);
        break;
        case 'l':
            qnode.move_base(is_all?'L':'l',liner,turn);
        break;
        case 'm':
            qnode.move_base(is_all?'M':'m',liner,turn);
        break;
        case ',':
            qnode.move_base(is_all?'<':',',liner,turn);
        break;
        case '.':
            qnode.move_base(is_all?'>':'.',liner,turn);
        break;
    }
}
//滑动条处理槽函数
void MainWindow::on_Slider_raw_valueChanged(int v)
{
    ui.label_raw->setText(QString::number(v));
}
//滑动条处理槽函数
void MainWindow::on_Slider_linear_valueChanged(int v)
{
    ui.label_linear->setText(QString::number(v));
}
//快捷指令删除按钮
void MainWindow::quick_cmd_remove()
{
    QTreeWidgetItem *curr=ui.treeWidget_quick_cmd->currentItem();
    //没有选择节点
    if(curr == nullptr) return;
    //获取父节点
    QTreeWidgetItem* parent=curr->parent();
    //如果当前节点就为父节点
    if(parent == nullptr)
    {
        ui.treeWidget_quick_cmd->takeTopLevelItem(ui.treeWidget_quick_cmd->indexOfTopLevelItem(curr));
        delete curr;
    }
    else{
        ui.treeWidget_quick_cmd->takeTopLevelItem(ui.treeWidget_quick_cmd->indexOfTopLevelItem(parent));
        delete parent;
    }


}
//快捷指令添加按钮
void MainWindow::quick_cmd_add()
{
    QWidget *w=new QWidget;
    //阻塞其他窗体
    w->setWindowModality(Qt::ApplicationModal);
    QLabel *name=new QLabel;
    name->setText("名称:");
    QLabel *content=new QLabel;
    content->setText("脚本:");
    QLineEdit *name_val=new QLineEdit;
    QTextEdit *shell_val=new QTextEdit;
    QPushButton *ok_btn=new QPushButton;
    ok_btn->setText("ok");
    ok_btn->setIcon(QIcon("://images/ok.png"));
    QPushButton *cancel_btn=new QPushButton;
    cancel_btn->setText("cancel");
    cancel_btn->setIcon(QIcon("://images/false.png"));
    QHBoxLayout *lay1=new QHBoxLayout;
    lay1->addWidget(name);
    lay1->addWidget(name_val);
    QHBoxLayout *lay2=new QHBoxLayout;
    lay2->addWidget(content);
    lay2->addWidget(shell_val);
    QHBoxLayout *lay3=new QHBoxLayout;
    lay3->addWidget(ok_btn);
    lay3->addWidget(cancel_btn);
    QVBoxLayout *v1=new QVBoxLayout;
    v1->addLayout(lay1);
    v1->addLayout(lay2);
    v1->addLayout(lay3);

    w->setLayout(v1);
    w->show();

    connect(ok_btn,&QPushButton::clicked,[this,w,name_val,shell_val]
    {
        this->add_quick_cmd(name_val->text(),shell_val->toPlainText());
        w->close();
    });
}
//向treeWidget添加快捷指令
void MainWindow::add_quick_cmd(QString name,QString val)
{
    if(name=="") return;
    QTreeWidgetItem *head=new QTreeWidgetItem(QStringList()<<name);
    this->ui.treeWidget_quick_cmd->addTopLevelItem(head);
    QCheckBox *check=new QCheckBox;
    //记录父子关系
    this->widget_to_parentItem_map[check]=head;
    //连接checkbox选中的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(quick_cmds_check_change(int)));
    this->ui.treeWidget_quick_cmd->setItemWidget(head,1,check);
    QTreeWidgetItem *shell_content=new QTreeWidgetItem(QStringList()<<"shell");
    QTextEdit *shell_val=new QTextEdit;
    shell_val->setMaximumWidth(150);
    shell_val->setMaximumHeight(40);
    head->addChild(shell_content);
    shell_val->setText(val);
    this->ui.treeWidget_quick_cmd->setItemWidget(shell_content,1,shell_val);
}
//快捷指令按钮处理的函数
void MainWindow::quick_cmds_check_change(int state)
{

    QCheckBox* check = qobject_cast<QCheckBox*>(sender());
    QTreeWidgetItem *parent=widget_to_parentItem_map[check];
    QString bash=((QTextEdit *)ui.treeWidget_quick_cmd->itemWidget(parent->child(0),1))->toPlainText();
    bool is_checked=state>1?true:false;
    if(is_checked)
    {
        quick_cmd=new QProcess;
        quick_cmd->start("bash");
        qDebug()<<bash;
        quick_cmd->write(bash.toLocal8Bit()+'\n');
        connect(quick_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(cmd_output()));
         connect(quick_cmd,SIGNAL(readyReadStandardError()),this,SLOT(cmd_error_output()));
    }
    else{


    }

}
//执行一些命令的回显
void MainWindow::cmd_output()
{

    ui.cmd_output->append(quick_cmd->readAllStandardOutput());
}
//执行一些命令的错误回显
void MainWindow::cmd_error_output()
{
    ui.cmd_output->append("<font color=\"#FF0000\">"+quick_cmd->readAllStandardError()+"</font> ");
}

//析构函数
MainWindow::~MainWindow() {


    if( base_cmd)
    {
        delete base_cmd;
        base_cmd = nullptr;
    }
    if(map_rviz_)
    {
        delete map_rviz_;
        map_rviz_ = nullptr;
    }


}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::inform(QString strdata)
{
    QMessageBox m_r;
    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.exec();
}
bool MainWindow::AskInform(QString strdata)
{
    QMessageBox m_r;

    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.addButton(tr("确认"), QMessageBox::ActionRole);
    m_r.addButton(tr("取消"), QMessageBox::ActionRole);

    int isok = m_r.exec();
    if (isok == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    //如果使用环境变量
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
            //showNoMasterMessage();
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
             ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
            ui.tab_manager->setTabEnabled(1,false);
            ui.tabWidget->setTabEnabled(1,false);
              ui.groupBox_3->setEnabled(false);
		} else {
            ui.tab_manager->setTabEnabled(1,true);
            ui.tabWidget->setTabEnabled(1,true);
            ui.groupBox_3->setEnabled(true);
            //初始化rviz
            initRviz();
			ui.button_connect->setEnabled(false);
              ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
              ui.label_statue_text->setStyleSheet("color:green;");
             ui.label_statue_text->setText("在线");
             //初始化视频订阅的显示
             initVideos();
             //显示话题列表
             initTopicList();
		}
    }
    //如果不使用环境变量
    else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
             ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
                ui.tab_manager->setTabEnabled(1,false);
                ui.tabWidget->setTabEnabled(1,false);
                  ui.groupBox_3->setEnabled(false);
            //showNoMasterMessage();
		} else {
            ui.tab_manager->setTabEnabled(1,true);
            ui.tabWidget->setTabEnabled(1,true);
            //初始化rviz
            initRviz();
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.groupBox_3->setEnabled(true);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui.label_statue_text->setStyleSheet("color:green;");
           ui.label_statue_text->setText("在线");
           //初始化视频订阅的显示
           initVideos();
           //显示话题列表
           initTopicList();
		}
	}

}
void MainWindow::initTopicList()
{
    ui.topic_listWidget->clear();
    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui.topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}
void MainWindow::refreashTopicList()
{
    initTopicList();
}
//当ros与master的连接断开时
void MainWindow::slot_rosShutdown()
{
    ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
     ui.label_statue_text->setStyleSheet("color:red;");
    ui.label_statue_text->setText("离线");
    ui.button_connect->setEnabled(true);
    ui.line_edit_master->setReadOnly(false);
    ui.line_edit_host->setReadOnly(false);
    ui.line_edit_topic->setReadOnly(false);
}
void MainWindow::slot_power(float p)
{
    ui.label_power->setText(QString::number(p).mid(0,5)+"V");
    double n=(p-10)/1.5;
    int value=n*100;
    ui.progressBar->setValue(value>100?100:value);
    //当电量过低时发出提示
    if(n*100<=20)
    {
         ui.progressBar->setStyleSheet("QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
          // QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！", QMessageBox::Yes , QMessageBox::Yes);
    }
    else{
        ui.progressBar->setStyleSheet("QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
    }
}
void MainWindow::slot_speed_x(double x)
{
    if(x>=0) ui.label_dir_x->setText("正向");
    else ui.label_dir_x->setText("反向");

    m_DashBoard_x->setValue(abs(x*100));
}
void MainWindow::slot_speed_y(double x)
{
    if(x>=0) ui.label_dir_y->setText("正向");
    else ui.label_dir_y->setText("反向");
    m_DashBoard_y->setValue(abs(x*100));
}
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    //QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_monitor");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }

    QSettings return_pos("return-position","cyrobot_monitor");
    ui.label_return_x->setText(return_pos.value("x",QString("0")).toString());
    ui.label_return_y->setText(return_pos.value("y",QString("0")).toString());
    ui.label_return_z->setText(return_pos.value("z",QString("0")).toString());
    ui.label_return_w->setText(return_pos.value("w",QString("0")).toString());

    //读取快捷指令的setting
    QSettings quick_setting("quick_setting","cyrobot_monitor");
    QStringList ch_key=quick_setting.childKeys();
    for(auto c:ch_key)
    {
        add_quick_cmd(c,quick_setting.value(c,QString("")).toString());
    }

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_monitor");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    //存下快捷指令的setting
    QSettings quick_setting("quick_setting","cyrobot_monitor");
    quick_setting.clear();
    for(int i=0;i<ui.treeWidget_quick_cmd->topLevelItemCount();i++)
    {
        QTreeWidgetItem *top=ui.treeWidget_quick_cmd->topLevelItem(i);
        QTextEdit *cmd_val=(QTextEdit *)ui.treeWidget_quick_cmd->itemWidget(top->child(0),1);
        quick_setting.setValue(top->text(0),cmd_val->toPlainText());
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor

///
/// \brief 增加topic
///
void cyrobot_monitor::MainWindow::on_pushButton_add_topic_clicked()
{
    if(!addtopic_form)
    {
        addtopic_form = new AddTopics();
        //阻塞其他窗体
        addtopic_form->setWindowModality(Qt::ApplicationModal);
        //绑定添加rviz话题信号
        connect(addtopic_form, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));
        addtopic_form->show();
    }
    else{
        QPoint p=addtopic_form->pos();
        delete addtopic_form;
        addtopic_form = new AddTopics();
        connect(addtopic_form, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));
        addtopic_form->show();
        addtopic_form->move(p.x(),p.y());
    }
}

///
/// \brief 删除topic
///
void cyrobot_monitor::MainWindow::on_pushButton_remove_topic_clicked()
{
    
}

///
/// \brief 重命名topic
///
void cyrobot_monitor::MainWindow::on_pushButton_rename_topic_clicked()
{
    
}

void cyrobot_monitor::MainWindow::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName = index.data().value<QString>();
    if (index.parent().row() == -1)   // Display 的根节点
    {
        if (index.row() > 1)
        {
            ui.pushButton_remove_topic->setEnabled(true);
            ui.pushButton_rename_topic->setEnabled(true);
        }
        else
        {
            ui.pushButton_remove_topic->setEnabled(false);
            ui.pushButton_rename_topic->setEnabled(false);
        }
    }
    else
    {
        ui.pushButton_remove_topic->setEnabled(false);
        ui.pushButton_rename_topic->setEnabled(false);
    }
}
