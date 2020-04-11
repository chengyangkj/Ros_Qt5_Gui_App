#include "../include/cyrobot_monitor/addtopics.h"
#include "ui_addtopics.h"
#include <QDebug>
AddTopics::AddTopics(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AddTopics)
{
    ui->setupUi(this);
    initUi();
    connect(ui->treeWidget,SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)),this,SLOT(slot_curritem_changed(QTreeWidgetItem *, QTreeWidgetItem *)));
}
void AddTopics::initUi()
{
    ui->treeWidget->setHeaderHidden(true);
    QTreeWidgetItem* parent=new QTreeWidgetItem(QStringList()<<"rviz");
    parent->setIcon(0,QIcon("://images/classes/Group.png"));
    ui->treeWidget->addTopLevelItem(parent);
    parent->setExpanded(true);


    Navigate=new QTreeWidgetItem(QStringList()<<"Navigate");
    Navigate->setIcon(0,QIcon("://images/Navigate.png"));
    parent->addChild(Navigate);

    Build_Map=new QTreeWidgetItem(QStringList()<<"Build Map");
    Build_Map->setIcon(0,QIcon("://images/Slam.png"));
    parent->addChild(Build_Map);

    Axes=new QTreeWidgetItem(QStringList()<<"Axes");
    Axes->setIcon(0,QIcon("://images/classes/Axes.png"));


    QTreeWidgetItem *Axesstatus=new QTreeWidgetItem(QStringList()<<"Statue:");
    Axesstatus->setIcon(0,QIcon("://images/ok.png"));
    parent->addChild(Axes);
    Axes->addChild(Axesstatus);
    //暂时隐藏子对像
    Axesstatus->setHidden(true);

     Camera=new QTreeWidgetItem(QStringList()<<"Camera");
    Camera->setIcon(0,QIcon("://images/classes/Camera.png"));
    parent->addChild(Camera);


     DepthCloud=new QTreeWidgetItem(QStringList()<<"DepthCloud");
    DepthCloud->setIcon(0,QIcon("://images/classes/DepthCloud.png"));
    parent->addChild(DepthCloud);

     Effort=new QTreeWidgetItem(QStringList()<<"Effort");
    Effort->setIcon(0,QIcon("://images/classes/Effort.png"));
    parent->addChild(Effort);

     FluidPressure=new QTreeWidgetItem(QStringList()<<"FluidPressure");
    FluidPressure->setIcon(0,QIcon("://images/classes/FluidPressure.png"));
    parent->addChild(FluidPressure);

     Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));
    parent->addChild(Grid);
    QTreeWidgetItem *gridstatus=new QTreeWidgetItem(QStringList()<<"Statue:");
    gridstatus->setIcon(0,QIcon("://images/ok.png"));
    Grid->addChild(gridstatus);
    gridstatus->setHidden(true);

     GridCells=new QTreeWidgetItem(QStringList()<<"GridCells");
    GridCells->setIcon(0,QIcon("://images/classes/GridCells.png"));
    parent->addChild(GridCells);

     Group=new QTreeWidgetItem(QStringList()<<"Group");
    Group->setIcon(0,QIcon("://images/classes/Group.png"));
    parent->addChild(Group);

     Illuminance=new QTreeWidgetItem(QStringList()<<"Illuminance");
    Illuminance->setIcon(0,QIcon("://images/classes/Illuminance.png"));
    parent->addChild(Illuminance);

     Image=new QTreeWidgetItem(QStringList()<<"Image");
    Image->setIcon(0,QIcon("://images/classes/Image.png"));
    parent->addChild(Image);

     InteractiveMarkers=new QTreeWidgetItem(QStringList()<<"InteractiveMarkers");
    InteractiveMarkers->setIcon(0,QIcon("://images/classes/InteractiveMarkers.png"));
    parent->addChild(InteractiveMarkers);

     LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
    LaserScan->setIcon(0,QIcon("://images/classes/LaserScan.png"));
    parent->addChild(LaserScan);
    QTreeWidgetItem* Laser_status=new QTreeWidgetItem(QStringList()<<"Status");
    Laser_status->setIcon(0,QIcon("://images/ok.png"));
    LaserScan->addChild(Laser_status);
    Laser_status->setHidden(true);
    QTreeWidgetItem* Laser_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    LaserScan->addChild(Laser_Topic);
    Laser_Topic->setHidden(true);


     Map=new QTreeWidgetItem(QStringList()<<"Map");
    Map->setIcon(0,QIcon("://images/classes/Map.png"));
    parent->addChild(Map);
    QTreeWidgetItem* Map_status=new QTreeWidgetItem(QStringList()<<"Status");
    Map_status->setIcon(0,QIcon("://images/ok.png"));
    Map->addChild(Map_status);
    Map_status->setHidden(true);
    QTreeWidgetItem* Map_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Map->addChild(Map_Topic);
    Map_Topic->setHidden(true);
    QTreeWidgetItem* Map_Alpha=new QTreeWidgetItem(QStringList()<<"Alpha");
    Map->addChild(Map_Alpha);
    Map_Alpha->setHidden(true);
    QTreeWidgetItem* Map_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map->addChild(Map_Color_Scheme);
    Map_Color_Scheme->setHidden(true);

     Marker=new QTreeWidgetItem(QStringList()<<"Marker");
    Marker->setIcon(0,QIcon("://images/classes/Marker.png"));
    parent->addChild(Marker);

     MarkerArray=new QTreeWidgetItem(QStringList()<<"MarkerArray");
    MarkerArray->setIcon(0,QIcon("://images/classes/MarkerArray.png"));
    parent->addChild(MarkerArray);

     Odometry=new QTreeWidgetItem(QStringList()<<"Odometry");
    Odometry->setIcon(0,QIcon("://images/classes/Odometry.png"));
    parent->addChild(Odometry);

     Path=new QTreeWidgetItem(QStringList()<<"Path");
    Path->setIcon(0,QIcon("://images/classes/Path.png"));
    parent->addChild(Path);

     PointCloud=new QTreeWidgetItem(QStringList()<<"PointCloud");
    PointCloud->setIcon(0,QIcon("://images/classes/PointCloud.png"));
    parent->addChild(PointCloud);

     PointCloud2=new QTreeWidgetItem(QStringList()<<"PointCloud2");
    PointCloud2->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
    parent->addChild(PointCloud2);

     PointStamped=new QTreeWidgetItem(QStringList()<<"PointStamped");
    PointStamped->setIcon(0,QIcon("://images/classes/PointStamped.png"));
    parent->addChild(PointStamped);

     Polygon=new QTreeWidgetItem(QStringList()<<"Polygon");
    Polygon->setIcon(0,QIcon("://images/classes/Polygon.png"));
    parent->addChild(Polygon);

     Pose=new QTreeWidgetItem(QStringList()<<"Pose");
    Pose->setIcon(0,QIcon("://images/classes/Pose.png"));
    parent->addChild(Pose);

     PoseArray=new QTreeWidgetItem(QStringList()<<"PoseArray");
    PoseArray->setIcon(0,QIcon("://images/classes/PoseArray.png"));
    parent->addChild(PoseArray);

     PoseWithCovariance=new QTreeWidgetItem(QStringList()<<"PoseWithCovariance");
    PoseWithCovariance->setIcon(0,QIcon("://images/classes/PoseWithCovariance.png"));
    parent->addChild(PoseWithCovariance);

     Range=new QTreeWidgetItem(QStringList()<<"Range");
    Range->setIcon(0,QIcon("://images/classes/Range.png"));
    parent->addChild(Range);

     RelativeHumidity=new QTreeWidgetItem(QStringList()<<"RelativeHumidity");
    RelativeHumidity->setIcon(0,QIcon("://images/classes/RelativeHumidity.png"));
    parent->addChild(RelativeHumidity);

     RobotModel=new QTreeWidgetItem(QStringList()<<"RobotModel");
    RobotModel->setIcon(0,QIcon("://images/classes/RobotModel.png"));
    parent->addChild(RobotModel);

     TF=new QTreeWidgetItem(QStringList()<<"TF");
    TF->setIcon(0,QIcon("://images/classes/TF.png"));
    parent->addChild(TF);

     Temperature=new QTreeWidgetItem(QStringList()<<"Temperature");
    Temperature->setIcon(0,QIcon("://images/classes/Temperature.png"));
    parent->addChild(Temperature);

     WrenchStamped=new QTreeWidgetItem(QStringList()<<"WrenchStamped");
    WrenchStamped->setIcon(0,QIcon("://images/classes/WrenchStamped.png"));
    parent->addChild(WrenchStamped);

     Imu=new QTreeWidgetItem(QStringList()<<"Imu");
    Imu->setIcon(0,QIcon("://images/classes/Imu.png"));
    parent->addChild(Imu);



}
void AddTopics::slot_curritem_changed(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
 if(current->text(0)=="rviz") ui->pushButton_ok->setEnabled(false);
 else ui->pushButton_ok->setEnabled(true);
 ui->lineEdit->setText(current->text(0));
 choose=current->clone();

}
AddTopics::~AddTopics()
{
    delete ui;  
    if(Axes) delete Axes;
    if(Camera)  delete Camera;
    if(DepthCloud)  delete DepthCloud;
    if(Effort)  delete Effort;
    if(FluidPressure)  delete FluidPressure;
    if(Grid)  delete Grid;
    if(GridCells)  delete GridCells;
    if(Group)  delete Group;
    if(Illuminance)  delete Illuminance;
    if(Image)  delete Image;
    if(InteractiveMarkers)  delete InteractiveMarkers;
    if(LaserScan)  delete LaserScan;
    if(Map)  delete Map;
    if(Marker)  delete Marker;
    if(MarkerArray)  delete MarkerArray;
    if(Odometry)  delete Odometry;
    if(Path)  delete Path;
    if(PointCloud)  delete PointCloud;
    if(PointCloud2)  delete PointCloud2;
    if(PointStamped)  delete PointStamped;
    if(Polygon)  delete Polygon;
    if(Pose)  delete Pose;
    if(PoseArray)  delete PoseArray;
    if(PoseWithCovariance)  delete PoseWithCovariance;
    if(Range)  delete Range;
    if(RelativeHumidity)  delete RelativeHumidity;
    if(RobotModel)  delete RobotModel;
    if(TF)  delete TF;
    if(Temperature)  delete Temperature;
    if(WrenchStamped)  delete WrenchStamped;
    if(Imu)  delete Imu;
}

void AddTopics::on_pushButton_ok_clicked()
{

//    //如果选择的是Navigate
//    if(choose->text(0)=="Navigate")
//    {
//      emit(Topic_choose(TF));
//      emit(Topic_choose(Map));

//    }
    emit(Topic_choose(choose));
    this->close();
}

void AddTopics::on_pushButton_cancel_clicked()
{
    this->close();
}
