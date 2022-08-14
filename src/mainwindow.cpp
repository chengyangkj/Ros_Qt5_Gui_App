#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "RobotAlgorithm.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    commNode=new rclcomm();
    connect(commNode,SIGNAL(emitTopicData(QString)),this,SLOT(onRecvData(QString)));
    QImage img;
    img.load("://images/foxy.jpg");
    ui->label_3->setPixmap(QPixmap::fromImage(img).scaled(ui->label_3->size(),Qt::KeepAspectRatio, Qt::SmoothTransformation));
    //初始化场景类
    m_qGraphicScene=new QGraphicsScene();
    m_qGraphicScene->clear();
    //初始化item
    m_roboItem = new roboItem();
    //视图中添加Item
    m_qGraphicScene->addItem(m_roboItem);
    //ui中的graphicsView添加场景
    ui->graphicsView->setScene(m_qGraphicScene);
    connect(commNode,SIGNAL(emitUpdateMap(QImage)),m_roboItem,SLOT(updateMap(QImage)));
    connect(commNode,SIGNAL(emitUpdateLocalCostMap(QImage)),m_roboItem,SLOT(updateLocalCostMap(QImage)));
    connect(commNode,SIGNAL(emitUpdateRobotPose(RobotPose)),m_roboItem,SLOT(updateRobotPose(RobotPose)));
    connect(commNode,SIGNAL(emitUpdateLaserPoint(QPolygonF)),m_roboItem,SLOT(updateLaserPoints(QPolygonF)));
    connect(commNode,SIGNAL(emitUpdatePath(QPolygonF)),m_roboItem,SLOT(updatePath(QPolygonF)));
//    connect(commNode,&rclcomm::emitUpdateMap,[this](QImage img){
//        m_roboItem->updateMap(img);
//    });
    connect(m_roboItem,SIGNAL(signalPub2DPose(QPointF,QPointF)),commNode,SLOT(pub2DPose(QPointF,QPointF)));
    connect(m_roboItem,SIGNAL(signalPub2DGoal(QPointF,QPointF)),commNode,SLOT(pub2DGoal(QPointF,QPointF)));
    commNode->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onRecvData(QString msg){
   ui->label_2->setText(msg);
}

void MainWindow::on_pushButton_clicked()
{

}
//显示图片
void MainWindow::on_pushButton_2_clicked()
{
    QImage img;
    img.load("://images/foxy.jpg");
    m_roboItem->updateImage(img);

}

void MainWindow::on_pushButton_3_clicked()
{
    QPolygonF point;
    point.push_back(QPointF(0,0));
    point.push_back(QPointF(1,1));
    point.push_back(QPointF(3,3));
    point.push_back(QPointF(5,6));
    m_roboItem->updatePoints(point);
}

void MainWindow::on_pushButton_4_clicked()
{
    QLine line(QPoint(0,0),QPoint(30,30));
    m_roboItem->updateLine(line);
}
//设置重定位点
void MainWindow::on_pushButton_5_clicked()
{
     m_roboItem->start2DPose();
}

void MainWindow::on_pushButton_6_clicked()
{
      m_roboItem->start2DGoal();
}
