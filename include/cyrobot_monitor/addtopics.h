#ifndef ADDTOPICS_H
#define ADDTOPICS_H

#include <QWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>
namespace Ui {
class AddTopics;
}

class AddTopics : public QWidget
{
    Q_OBJECT

public:
    explicit AddTopics(QWidget *parent = 0);
    QTreeWidgetItem *choose;
    ~AddTopics();
signals:
    void Topic_choose(QTreeWidgetItem *choose);
private slots:
    void on_pushButton_cancel_clicked();

private slots:
    void on_pushButton_ok_clicked();

private slots:
    void slot_curritem_changed(QTreeWidgetItem *current, QTreeWidgetItem *previous);

private:
    Ui::AddTopics *ui;
    void initUi();
    QCheckBox *checkbox;
    QTreeWidgetItem *Navigate=NULL;
    QTreeWidgetItem *Build_Map=NULL;
    QTreeWidgetItem *Axes=NULL;
    QTreeWidgetItem *Camera=NULL;
    QTreeWidgetItem *DepthCloud=NULL;
    QTreeWidgetItem *Effort=NULL;
    QTreeWidgetItem *FluidPressure=NULL;
    QTreeWidgetItem *Grid=NULL;
    QTreeWidgetItem *GridCells=NULL;
    QTreeWidgetItem *Group=NULL;
    QTreeWidgetItem *Illuminance=NULL;
    QTreeWidgetItem *Image=NULL;
    QTreeWidgetItem *InteractiveMarkers=NULL;
    QTreeWidgetItem *LaserScan=NULL;
    QTreeWidgetItem *Map=NULL;
    QTreeWidgetItem *Marker=NULL;
    QTreeWidgetItem *MarkerArray=NULL;
    QTreeWidgetItem *Odometry=NULL;
    QTreeWidgetItem *Path=NULL;
    QTreeWidgetItem *PointCloud=NULL;
    QTreeWidgetItem *PointCloud2=NULL;
    QTreeWidgetItem *PointStamped=NULL;
    QTreeWidgetItem *Polygon=NULL;
    QTreeWidgetItem *Pose=NULL;
    QTreeWidgetItem *PoseArray=NULL;
    QTreeWidgetItem *PoseWithCovariance=NULL;
    QTreeWidgetItem *Range=NULL;
    QTreeWidgetItem *RelativeHumidity=NULL;
    QTreeWidgetItem *RobotModel=NULL;
    QTreeWidgetItem *TF=NULL;
    QTreeWidgetItem *Temperature=NULL;
    QTreeWidgetItem *WrenchStamped=NULL;
    QTreeWidgetItem *Imu=NULL;
};

#endif // ADDTOPICS_H
