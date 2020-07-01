#include "../include/cyrobot_monitor_simple/mainform.h"
#include "ui_mainform.h"

MainForm::MainForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainForm)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog|Qt::FramelessWindowHint);
    //设置窗口全屏
    this->setWindowState(this->windowState() ^ Qt::WindowFullScreen);
    //获取qml的object
    qmlRoot=(QObject*)ui->quickWidget->rootObject();
    //初始化qnode
    int argc;
    char** argv;
    qnode=new cyrobot_monitor_simple::QNode(argc,argv);
    bool ok= qnode->init();
//    qDebug()<<ok;
    //***链接相关信号***************************************************
    //qml相关信号
    connect(qnode,SIGNAL(speed_x(double)),qmlRoot,SIGNAL(speed_x_signal(double)));
//    connect(qnode,SIGNAL(speed_y(double)),qmlRoot,SIGNAL(speed_y_signal(double)));
    connect(qmlRoot,SIGNAL(quitSignal()),this,SLOT(close()));
}
void MainForm::closeEvent(QCloseEvent *event)
{
//	WriteSettings();
    QWidget::closeEvent(event);
}
MainForm::~MainForm()
{
    delete ui;
}
