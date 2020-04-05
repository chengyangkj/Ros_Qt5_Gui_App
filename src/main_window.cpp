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
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    initRviz();
    //链接connect
    connections();

}
//初始化UI
void MainWindow::initUis()
{

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
    QLineEdit *colorval=new QLineEdit("255;255;255");
    colorval->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(bcolor,1,colorval);

    QSpinBox *framerateval=new QSpinBox();
    framerateval->setStyleSheet("border:none");
    framerateval->setMaximumWidth(150);
    framerateval->setRange(10,50);
    QTreeWidgetItem* framerate=new QTreeWidgetItem(QStringList()<<"Frame Rate");
    Global->addChild(framerate);
    ui.treeWidget_rviz->setItemWidget(framerate,1,framerateval);

    //grid
    QTreeWidgetItem *Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));

    ui.treeWidget_rviz->addTopLevelItem(Grid);
    Grid->setExpanded(true);
    QCheckBox* gridcheck=new QCheckBox;
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
    QSpinBox* Plan_Cell_Count_Value=new QSpinBox();
    Grid->addChild(Plan_Cell_Count);
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


}
void MainWindow::initRviz()
{

qrviz=new QRviz_widget(ui.widget_rviz);
qrviz->showFullScreen();
qrviz->show();

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
    //绑定快捷按钮相关函数
    connect(ui.laser_btn,SIGNAL(clicked()),this,SLOT(quick_cmds()));
    connect(ui.basecontrol_btn,SIGNAL(clicked()),this,SLOT(quick_cmds()));
   //绑定slider的函数
   connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_raw_valueChanged(int)));
   connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_linear_valueChanged(int)));
   //绑定速度控制按钮
   connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_back,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
   connect(ui.pushButton_backr,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
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
             tree_rviz_keys[controls]=top;
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
        tree_rviz_keys[check]=top;
        connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));


    }
    //connect(ui.treeWidget_rviz,SIGNAL(itemChanged(QTreeWidgetItem*,int)),this,SLOT(slot_treewidget_item_value_change(QTreeWidgetItem*,int)));
}
//treewidget的checkbox是否选中槽函数
void MainWindow::slot_treewidget_item_check_change(int is_check)
{
    QCheckBox* sen = (QCheckBox*)sender();
    qDebug()<<"check:"<<is_check<<"parent:"<<tree_rviz_keys[sen]->text(0)<<"地址："<<tree_rviz_keys[sen];
}
//treewidget 的值槽函数
void MainWindow::slot_treewidget_item_value_change(QString value)
{
    QWidget* sen = (QWidget*)sender();
    qDebug()<<sen->metaObject()->className()<<"parent:"<<tree_rviz_keys[(QWidget*)sen]->text(0);
//    qDebug()<<value;
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
    QCheckBox *check=new QCheckBox();
    ui.treeWidget_rviz->setItemWidget(choose,1,check);


    //记录父子关系
    tree_rviz_keys[check]=choose;
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));
}
//左工具栏索引改变
void MainWindow::slot_tab_manage_currentChanged(int index)
{
    switch (index) {
    case 0:
        ui.tabWidget->setCurrentIndex(0);
        break;
    case 1:

        ui.tabWidget->setCurrentIndex(1);
        break;
    case 2:
         ui.tabWidget->setCurrentIndex(2);
        break;

    }
}
//右工具栏索引改变
void MainWindow::slot_tab_Widget_currentChanged(int index)
{
    switch (index) {
    case 0:
        ui.tab_manager->setCurrentIndex(0);
        break;
    case 1:
        ui.tab_manager->setCurrentIndex(1);
        break;
    case 2:
        ui.tab_manager->setCurrentIndex(2);
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
//快捷指令按钮处理的函数
void MainWindow::quick_cmds()
{
    QPushButton* btn = qobject_cast<QPushButton*>(sender());
   if(btn->objectName()=="laser_btn")
   {
//       if(laser_cmd==NULL)
//       {
//           laser_cmd = new QProcess(this);
//           laser_cmd->start("bash");

//           //等待启动完成
//           laser_cmd->waitForStarted();
//           laser_cmd->write(ui.laer_cmd_text->toPlainText().toLocal8Bit() + '\n');
//           connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(cmd_output()));
//           btn->setText("关闭激光雷达");
//       }
//       else{
//         laser_cmd->write("exit\n");
//         laser_cmd->write("exit\n");
//         laser_cmd->close();
//         laser_cmd->waitForFinished();
//         btn->setText("开启激光雷达");
//         delete laser_cmd;->child(j)[1]
//         laser_cmd=NULL;
//       }

   }
   else if(btn->objectName()=="basecontrol_btn")
   {
//       if(base_cmd==NULL)
//       {
//           base_cmd=new QProcess();
//           base_cmd->start("bash");
//           base_cmd->waitForStarted();
//           base_cmd->wr->child(j)[1]ite(ui.base_cmd_text->toPlainText().toLocal8Bit()+'\n');
//          // base_cmd->execute("gnome-terminal");

//           btn->setText("关闭底盘控制");
//       }
//       else{
//           //base_cmd->write("exit\n");

//           base_cmd->close();
//           base_cmd->waitForFinished();
//           btn->setText("开启底盘控制");
//           delete base_cmd;
//           base_cmd=NULL;
//       }
   }
}
//执行一些命令的回显
void MainWindow::cmd_output()
{
    if(laser_cmd!=NULL)
    {
        ui.cmd_output->append(laser_cmd->readAllStandardOutput().data());
    }
}

//析构函数
MainWindow::~MainWindow() {

    if(laser_cmd!=NULL)
    {
        laser_cmd->close();
        laser_cmd->waitForFinished();
        delete laser_cmd;
    }
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
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
            //showNoMasterMessage();
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
             ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
		} else {
			ui.button_connect->setEnabled(false);
              ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
              ui.label_statue_text->setStyleSheet("color:green;");
             ui.label_statue_text->setText("在线");
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
             ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
            //showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui.label_statue_text->setStyleSheet("color:green;");
           ui.label_statue_text->setText("在线");
		}
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
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
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

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //关闭时释放内存
    this->setAttribute(Qt::WA_DeleteOnClose);
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor



