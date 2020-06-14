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
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    //链接connect
    connections();

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

    ui.treeWidget_rviz->setEnabled(false);
    m_DashBoard_x =new CCtrlDashBoard(ui.widget_speed_x);
    m_DashBoard_x->setGeometry(ui.widget_speed_x->rect());
    m_DashBoard_x->setValue(0);
    m_DashBoard_y =new CCtrlDashBoard(ui.widget_speed_y);
    m_DashBoard_y->setGeometry(ui.widget_speed_y->rect());
    m_DashBoard_y->setValue(0);

    ui.tab_manager->setCurrentIndex(0);
    ui.tabWidget->setCurrentIndex(0);
    //treeWidget_rviz
    ui.treeWidget_rviz->setWindowTitle("Displays");
    ui.treeWidget_rviz->setWindowIcon(QIcon("://images/classes/Displays.svg"));
    //header 设置
    ui.treeWidget_rviz->setHeaderHidden(true);
    ui.treeWidget_rviz->setHeaderLabels(QStringList()<<"key"<<"value");

    //Global options
    QTreeWidgetItem *Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));

    QTreeWidgetItem* FixedFrame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    Global->addChild(FixedFrame);

    ui.treeWidget_rviz->addTopLevelItem(Global);
    Global->setExpanded(true);
    //添加combox控件
    QComboBox *frame=new QComboBox();
    frame->addItem("map");
    frame->setEditable(true);
    frame->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(FixedFrame,1,frame);


    QTreeWidgetItem* bcolor=new QTreeWidgetItem(QStringList()<<"Background Color");
    Global->addChild(bcolor);
    //添加lineedit控件
    QLineEdit *colorval=new QLineEdit("48;48;48");
    colorval->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(bcolor,1,colorval);

    QSpinBox *framerateval=new QSpinBox();
    framerateval->setStyleSheet("border:none");
    framerateval->setMaximumWidth(150);
    framerateval->setRange(10,50);
    framerateval->setValue(30);
    QTreeWidgetItem* framerate=new QTreeWidgetItem(QStringList()<<"Frame Rate");
    Global->addChild(framerate);
    ui.treeWidget_rviz->setItemWidget(framerate,1,framerateval);

    //grid
    QTreeWidgetItem *Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));

    ui.treeWidget_rviz->addTopLevelItem(Grid);
    Grid->setExpanded(true);
    QCheckBox* gridcheck=new QCheckBox;
    gridcheck->setChecked(true);
    ui.treeWidget_rviz->setItemWidget(Grid,1,gridcheck);

    QTreeWidgetItem *Grid_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
    Grid_Status->setIcon(0,QIcon("://images/ok.png"));
    Grid->addChild(Grid_Status);
    QLabel *Grid_Status_Value=new QLabel("ok");
    Grid_Status_Value->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(Grid_Status,1,Grid_Status_Value);

    QTreeWidgetItem* Reference_Frame=new QTreeWidgetItem(QStringList()<<"Reference Frame");
    QComboBox* Reference_Frame_Value=new QComboBox();
    Grid->addChild(Reference_Frame);
    Reference_Frame_Value->setMaximumWidth(150);
    Reference_Frame_Value->setEditable(true);
    Reference_Frame_Value->addItem("<Fixed Frame>");
    ui.treeWidget_rviz->setItemWidget(Reference_Frame,1,Reference_Frame_Value);

    QTreeWidgetItem* Plan_Cell_Count=new QTreeWidgetItem(QStringList()<<"Plan Cell Count");
    Grid->addChild(Plan_Cell_Count);
    QSpinBox* Plan_Cell_Count_Value=new QSpinBox();

    Plan_Cell_Count_Value->setMaximumWidth(150);
    Plan_Cell_Count_Value->setRange(1,100);
    Plan_Cell_Count_Value->setValue(10);
    ui.treeWidget_rviz->setItemWidget(Plan_Cell_Count,1,Plan_Cell_Count_Value);

    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    QLineEdit* Grid_Color_Value=new QLineEdit();
    Grid_Color_Value->setMaximumWidth(150);
    Grid->addChild(Grid_Color);

    Grid_Color_Value->setText("160;160;160");
    ui.treeWidget_rviz->setItemWidget(Grid_Color,1,Grid_Color_Value);

    //qucik treewidget
    ui.treeWidget_quick_cmd->setHeaderLabels(QStringList()<<"key"<<"values");
    ui.treeWidget_quick_cmd->setHeaderHidden(true);

}
void MainWindow::initRviz()
{
map_rviz=new QRviz(ui.verticalLayout_build_map,"qrviz");
QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
QString Reference_text=Global_op->currentText();
map_rviz->Display_Grid(true,Reference_text,10,QColor(160,160,160));



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
    //添加rviz话题的按钮
    connect(ui.pushButton_add_topic,SIGNAL(clicked()),this,SLOT(slot_add_topic_btn()));
    //treewidget的值改变的槽函数
    //绑定treeiew所有控件的值改变函数
    for(int i=0;i<ui.treeWidget_rviz->topLevelItemCount();i++)
    {
        //top 元素
        QTreeWidgetItem *top=ui.treeWidget_rviz->topLevelItem(i);
//        qDebug()<<top->text(0)<<endl;
        for(int j=0;j<top->childCount();j++)
        {

             //获取该WidgetItem的子节点
             QTreeWidgetItem* tmp= top->child(j);
             QWidget* controls=ui.treeWidget_rviz->itemWidget(tmp,1);
//             qDebug()<<controls;
             //将当前控件对象和父级对象加入到map中
             widget_to_parentItem_map[controls]=top;
             //判断这些widget的类型 并分类型进行绑定槽函数
             if(QString(controls->metaObject()->className())=="QComboBox")
             {
                 connect(controls,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
             }
             else if(QString(controls->metaObject()->className())=="QLineEdit")
              {
                 connect(controls,SIGNAL(textChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
               }
             else if(QString(controls->metaObject()->className())=="QSpinBox")
             {
                 connect(controls,SIGNAL(valueChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
             }
        }
    }
    //绑定treeview checkbox选中事件
   // stateChanged

    for(int i=0;i<ui.treeWidget_rviz->topLevelItemCount();i++)
    {
        //top 元素
        QTreeWidgetItem *top=ui.treeWidget_rviz->topLevelItem(i);
        QWidget *check=ui.treeWidget_rviz->itemWidget(top,1);
        //记录父子关系
        widget_to_parentItem_map[check]=top;
        connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));


    }
    //connect(ui.treeWidget_rviz,SIGNAL(itemChanged(QTreeWidgetItem*,int)),this,SLOT(slot_treewidget_item_value_change(QTreeWidgetItem*,int)));
}
//设置界面
void MainWindow::slot_setting_frame()
{
    if(set!=NULL)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else{
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
    if(media_player!=NULL)
     {
         delete media_player;
         media_player=NULL;
     }
     media_player=new QSoundEffect;
     media_player->setSource(QUrl::fromLocalFile("://media/refresh_return.wav"));
     media_player->play();

}
//返航
void MainWindow::slot_return_point()
{
    qnode.set_goal(ui.label_frame->text(),ui.label_return_x->text().toDouble(),ui.label_return_y->text().toDouble(),ui.label_return_z->text().toDouble(),ui.label_return_w->text().toDouble());
    if(media_player!=NULL)
       {
           delete media_player;
           media_player=NULL;
       }
       media_player=new QSoundEffect;
       media_player->setSource(QUrl::fromLocalFile("://media/start_return.wav"));
       media_player->play();
}
//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz->Set_Pos();
// ui.label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz->Set_Goal();
//  ui.label_map_msg->setText("请在地图中选择机器人导航的目标位置");
}
void MainWindow::slot_move_camera_btn()
{
    map_rviz->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz->Set_Select();
}
//treewidget的checkbox是否选中槽函数
void MainWindow::slot_treewidget_item_check_change(int is_check)
{
    QCheckBox* sen = (QCheckBox*)sender();
    qDebug()<<"check:"<<is_check<<"parent:"<<widget_to_parentItem_map[sen]->text(0)<<"地址："<<widget_to_parentItem_map[sen];
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString dis_name=widget_to_parentItem_map[sen]->text(0);
    bool enable=is_check>1?true:false;
    if(dis_name=="Grid")
    {

        QLineEdit *Color_text=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        QString co=Color_text->text();
        QStringList colorList=co.split(";");
        QColor cell_color=QColor(colorList[0].toInt(),colorList[1].toInt(),colorList[2].toInt());

        QComboBox *Reference_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QString Reference_text=Reference_box->currentText();
        if(Reference_box->currentText()=="<Fixed Frame>")
        {
            QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
            Reference_text=Global_op->currentText();
        }
        QSpinBox *plan_cell_count=(QSpinBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        map_rviz->Display_Grid(enable,Reference_text,plan_cell_count->text().toInt(),cell_color);

    }
    else if(dis_name=="Map")
    {
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QLineEdit *alpha=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        QComboBox *scheme=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
//        qDebug()<<topic_box->currentText()<<alpha->text()<<scheme->currentText();
        map_rviz->Display_Map(enable,topic_box->currentText(),alpha->text().toDouble(),scheme->currentText());
    }
    else if(dis_name=="LaserScan")
    {
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_LaserScan(enable,topic_box->currentText());
    }
    else if(dis_name=="Navigate")
    {
        QComboBox* Global_map=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(0)->child(0)->child(0),1);
        QComboBox* Global_plan=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(0)->child(1)->child(0),1);
        QComboBox* Local_map=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1)->child(0)->child(0),1);
        QComboBox* Local_plan=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1)->child(1)->child(0),1);

        map_rviz->Display_Navigate(enable,Global_map->currentText(),Global_plan->currentText(),Local_map->currentText(),Local_plan->currentText());
    }
    else if(dis_name=="RobotModel")
    {
        map_rviz->Display_RobotModel(enable);
    }
}
//treewidget 的值改变槽函数
void MainWindow::slot_treewidget_item_value_change(QString value)
{

    QWidget* sen = (QWidget*)sender();
    qDebug()<<sen->metaObject()->className()<<"parent:"<<widget_to_parentItem_map[sen]->text(0);
    qDebug()<<value;
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString Dis_Name=widget_to_parentItem_map[sen]->text(0);

//    qDebug()<<"sdad"<<enable;
    //判断每种显示的类型
    if(Dis_Name=="Grid")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QLineEdit *Color_text=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        QString co=Color_text->text();
        QStringList colorList=co.split(";");
        QColor cell_color=QColor(colorList[0].toInt(),colorList[1].toInt(),colorList[2].toInt());

        QComboBox *Reference_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QString Reference_text=Reference_box->currentText();
        if(Reference_box->currentText()=="<Fixed Frame>")
        {
            QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
            Reference_text=Global_op->currentText();
        }
        QSpinBox *plan_cell_count=(QSpinBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        map_rviz->Display_Grid(enable,Reference_text,plan_cell_count->text().toInt(),cell_color);

    }
    else if(Dis_Name=="Global Options")
    {
        QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
        QString Reference_text=Global_op->currentText();
        QLineEdit *back_color=(QLineEdit *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(1),1);
        QStringList coList=back_color->text().split(";");
        QColor colorBack=QColor(coList[0].toInt(),coList[1].toInt(),coList[2].toInt());
        QSpinBox *FrameRaBox=(QSpinBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(2),1);
        map_rviz->SetGlobalOptions(Reference_text,colorBack,FrameRaBox->value());
    }
    else if(Dis_Name=="Map")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QLineEdit *alpha=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        QComboBox *scheme=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        qDebug()<<topic_box->currentText()<<alpha->text()<<scheme->currentText();
        map_rviz->Display_Map(enable,topic_box->currentText(),alpha->text().toDouble(),scheme->currentText());
    }
    else if(Dis_Name=="LaserScan")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_LaserScan(enable,topic_box->currentText());
    }


}
//rviz添加topic的槽函数
void MainWindow::slot_add_topic_btn()
{

    if(!addtopic_form)
    {
        addtopic_form=new AddTopics();
        //阻塞其他窗体
        addtopic_form->setWindowModality(Qt::ApplicationModal);
        //绑定添加rviz话题信号
        connect(addtopic_form,SIGNAL(Topic_choose(QTreeWidgetItem *)),this,SLOT(slot_choose_topic(QTreeWidgetItem *)));
        addtopic_form->show();
    }
    else{
        QPoint p=addtopic_form->pos();
        delete addtopic_form;
        addtopic_form=new AddTopics();
        connect(addtopic_form,SIGNAL(Topic_choose(QTreeWidgetItem *)),this,SLOT(slot_choose_topic(QTreeWidgetItem *)));
        addtopic_form->show();
        addtopic_form->move(p.x(),p.y());
    }
}
//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose)
{
    ui.treeWidget_rviz->addTopLevelItem(choose);
    //添加是否启用的checkbox
    QCheckBox *check=new QCheckBox();
    ui.treeWidget_rviz->setItemWidget(choose,1,check);
    //记录父子关系
    widget_to_parentItem_map[check]=choose;
    //绑定checkbox的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));
    //添加状态的对应关系到map
    tree_rviz_stues[choose->text(0)]=choose->child(0);

    if(choose->text(0)=="Map")
    {
        QComboBox *Map_Topic=new QComboBox();
        Map_Topic->addItem("map");
        Map_Topic->setEditable(true);
        Map_Topic->setMaximumWidth(150);
        widget_to_parentItem_map[Map_Topic]=choose;
        ui.treeWidget_rviz->setItemWidget(choose->child(1),1,Map_Topic);
        //绑定值改变了的事件
        connect(Map_Topic,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        QLineEdit *map_alpha=new QLineEdit;
        map_alpha->setMaximumWidth(150);
        map_alpha->setText("0.7");
        widget_to_parentItem_map[map_alpha]=choose;
        //绑定值改变了的事件
        connect(map_alpha,SIGNAL(textChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->setItemWidget(choose->child(2),1,map_alpha);

        QComboBox *Map_Scheme=new QComboBox;
        Map_Scheme->setMaximumWidth(150);
        Map_Scheme->addItems(QStringList()<<"map"<<"costmap"<<"raw");
        widget_to_parentItem_map[Map_Scheme]=choose;
        //绑定值改变了的事件
        connect(Map_Scheme,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->setItemWidget(choose->child(3),1,Map_Scheme);
    }
    else if(choose->text(0)=="LaserScan")
    {

        QComboBox *Laser_Topic=new QComboBox;
        Laser_Topic->setMaximumWidth(150);
        Laser_Topic->addItem("scan");
        Laser_Topic->setEditable(true);
        widget_to_parentItem_map[Laser_Topic]=choose;
        ui.treeWidget_rviz->setItemWidget(choose->child(1),1,Laser_Topic);
        //绑定值改变了的事件
        connect(Laser_Topic,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

    }
    else if(choose->text(0)=="Navigate")
    {
        //Global Map
        QTreeWidgetItem *Global_map=new QTreeWidgetItem(QStringList()<<"Global Map");
        Global_map->setIcon(0,QIcon("://images/classes/Group.png"));
        choose->addChild(Global_map);
        QTreeWidgetItem *Costmap=new QTreeWidgetItem(QStringList()<<"Costmap");
        Costmap->setIcon(0,QIcon("://images/classes/Map.png"));
        QTreeWidgetItem *Costmap_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        Costmap->addChild(Costmap_topic);
        QComboBox *Costmap_Topic_Vel=new QComboBox();
        Costmap_Topic_Vel->setEditable(true);
        Costmap_Topic_Vel->setMaximumWidth(150);
        Costmap_Topic_Vel->addItem("/move_base/global_costmap/costmap");
        ui.treeWidget_rviz->setItemWidget(Costmap_topic,1,Costmap_Topic_Vel);
        Global_map->addChild(Costmap);
        //绑定子父关系
        widget_to_parentItem_map[Costmap_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Costmap_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

        QTreeWidgetItem* CostMap_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
        CostMap_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
       Global_map->addChild(CostMap_Planner);

       QTreeWidgetItem* CostMap_Planner_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* Costmap_Planner_Topic_Vel=new QComboBox();
        Costmap_Planner_Topic_Vel->setMaximumWidth(150);
        Costmap_Planner_Topic_Vel->addItem("/move_base/DWAPlannerROS/global_plan");
        Costmap_Planner_Topic_Vel->setEditable(true);
        CostMap_Planner->addChild(CostMap_Planner_topic);
        ui.treeWidget_rviz->setItemWidget(CostMap_Planner_topic,1,Costmap_Planner_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[Costmap_Planner_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Costmap_Planner_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));



        //Local Map
        QTreeWidgetItem *Local_map=new QTreeWidgetItem(QStringList()<<"Local Map");
        Local_map->setIcon(0,QIcon("://images/classes/Group.png"));
        choose->addChild(Local_map);

        QTreeWidgetItem *Local_Costmap=new QTreeWidgetItem(QStringList()<<"Costmap");
        Local_Costmap->setIcon(0,QIcon("://images/classes/Map.png"));
        Local_map->addChild(Local_Costmap);

        QTreeWidgetItem *local_costmap_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        Local_Costmap->addChild(local_costmap_topic);
        QComboBox *local_costmap_topic_vel=new QComboBox();
        local_costmap_topic_vel->setEditable(true);
        local_costmap_topic_vel->setMaximumWidth(150);
        local_costmap_topic_vel->addItem("/move_base/local_costmap/costmap");
        ui.treeWidget_rviz->setItemWidget(local_costmap_topic,1,local_costmap_topic_vel);

        //绑定子父关系
        widget_to_parentItem_map[local_costmap_topic_vel]=choose;
        //绑定值改变了的事件
        connect(local_costmap_topic_vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

        QTreeWidgetItem* LocalMap_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
        LocalMap_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
       Local_map->addChild(LocalMap_Planner);

       QTreeWidgetItem* Local_Planner_topic=new QTreeWidgetItem(QStringList()<<"Topic");

        QComboBox* Local_Planner_Topic_Vel=new QComboBox();
        Local_Planner_Topic_Vel->setMaximumWidth(150);
        Local_Planner_Topic_Vel->addItem("/move_base/DWAPlannerROS/local_plan");
        Local_Planner_Topic_Vel->setEditable(true);
        LocalMap_Planner->addChild(Local_Planner_topic);
        ui.treeWidget_rviz->setItemWidget(Local_Planner_topic,1, Local_Planner_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[Local_Planner_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Local_Planner_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        //CostCloud
        QTreeWidgetItem* CostCloud=new QTreeWidgetItem(QStringList()<<"Cost Cloud");
        CostCloud->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
        Local_map->addChild(CostCloud);
        QTreeWidgetItem *CostCloud_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* CostCloud_Topic_Vel=new QComboBox();
        CostCloud_Topic_Vel->setMaximumWidth(150);
        CostCloud_Topic_Vel->setEditable(true);
        CostCloud_Topic_Vel->addItem("/move_base/DWAPlannerROS/cost_cloud");
        CostCloud->addChild(CostCloud_Topic);
        ui.treeWidget_rviz->setItemWidget(CostCloud_Topic,1,CostCloud_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[CostCloud_Topic_Vel]=CostCloud;
        //绑定值改变了的事件
        connect(CostCloud_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        //Trajectory Cloud
        QTreeWidgetItem* TrajectoryCloud=new QTreeWidgetItem(QStringList()<<"Trajectory Cloud");
        TrajectoryCloud->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
        Local_map->addChild(TrajectoryCloud);
        QTreeWidgetItem *TrajectoryCloud_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* TrajectoryCloud_Topic_Vel=new QComboBox();
        TrajectoryCloud_Topic_Vel->setMaximumWidth(150);
        TrajectoryCloud_Topic_Vel->setEditable(true);
        TrajectoryCloud_Topic_Vel->addItem("/move_base/DWAPlannerROS/trajectory_cloud");
        TrajectoryCloud->addChild(TrajectoryCloud_Topic);
        ui.treeWidget_rviz->setItemWidget(TrajectoryCloud_Topic,1,TrajectoryCloud_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[TrajectoryCloud_Topic_Vel]=TrajectoryCloud;
        //绑定值改变了的事件
        connect(TrajectoryCloud_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->addTopLevelItem(choose);
        choose->setExpanded(true);
    }
    else if(choose->text(0)=="TF")
    {
        ui.treeWidget_rviz->addTopLevelItem(choose);
    }


    //默认选中
    check->setChecked(true);
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
    float liner=ui.horizontalSlider_linear->value()*0.01;
    float turn=ui.horizontalSlider_raw->value()*0.01;
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
    if(curr==NULL) return;
    //获取父节点
    QTreeWidgetItem* parent=curr->parent();
    //如果当前节点就为父节点
    if(parent==NULL)
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
        base_cmd=NULL;
    }
    if(map_rviz)
    {
        delete map_rviz;
        map_rviz=NULL;
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
             ui.treeWidget_rviz->setEnabled(false);

		} else {

            //初始化rviz
            initRviz();
            ui.treeWidget_rviz->setEnabled(true);
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
                ui.treeWidget_rviz->setEnabled(false);
            //showNoMasterMessage();
		} else {

            //初始化rviz
            initRviz();
            ui.treeWidget_rviz->setEnabled(true);
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
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
    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui.topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
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
    //关闭时释放内存
    this->setAttribute(Qt::WA_DeleteOnClose);
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor



