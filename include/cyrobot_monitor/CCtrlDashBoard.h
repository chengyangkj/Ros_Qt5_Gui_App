#ifndef CCTRLDASHBOARD_H
#define CCTRLDASHBOARD_H

#include <QWidget>

class CCtrlDashBoard : public QWidget
{
    Q_OBJECT
public:
    enum StyleType {
        ST_DEFAULT=0,
        ST_ARCBAR
    };
    explicit CCtrlDashBoard(QWidget *parent = nullptr, StyleType type=ST_DEFAULT);

    void setValue(qreal value){
        m_DashValue = value;
        update();
    }
    void setBackGroundColor(QColor color){
        m_BgColor=color;
        update();
    }
    void setFrontColor(QColor color){
        m_FrontColor=color;
        update();
    }
    void setBorderColor(QColor color){
        m_BorderColor=color;
        update();
    }
    void setUnitString(QString str){
        m_StrUnit=str;
        update();
    }

    void drawBackGround(QPainter *painter, qreal hlafWidth);
    void drawScaleDials(QPainter *painter, qreal hlafWidth);
    void drawIndicator(QPainter *painter, qreal hlafWidth);
    void drawIndicatorBar(QPainter *painter, qreal hlafWidth);
signals:

public slots:
protected:
    virtual void paintEvent(QPaintEvent * event);

private:
    int m_StartAngle;
    int m_EndAngle;
    int m_StyleType;

    qreal m_LineLength;
    qreal m_DashValue;
    qreal m_MaxValue;
    qreal m_MinValue;
    qreal m_DashNum;

    QColor m_BgColor;
    QColor m_FrontColor;
    QColor m_BorderColor;
    QString m_StrUnit;

    qreal m_MaxBorderRadius;
    qreal m_MinBorderRadius;
    qreal m_DialsRadius;
};

#endif // CCTRLDASHBOARD_H
