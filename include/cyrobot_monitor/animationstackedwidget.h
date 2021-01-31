#ifndef ANIMATIONSTACKEDWIDGET_H
#define ANIMATIONSTACKEDWIDGET_H

#include <QPainter>
#include <QVariant>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QPropertyAnimation>

class AnimationStackedWidget : public QStackedWidget
{
    Q_OBJECT
public:
    enum AnimationType
    {
        TopToBottom,
        BottomToTop,
        LeftToRight,
        RightToLeft
    };

    explicit AnimationStackedWidget(QWidget *parent = 0);
	virtual ~AnimationStackedWidget();

	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);

	void start(int index);
	void setNext(bool);
    void setLength(int length, AnimationType type);
	void setWidgets(QList<QWidget*>);
	void setPos(QList<QPoint>);
private slots:
    void valueChanged(const QVariant &value);
    void animationFinished();
signals:
	void sigGetWidgets(int);
protected:
    virtual void paintEvent(QPaintEvent *event);
    void renderPreviousWidget(QPainter &painter, QTransform &transform);
    void renderCurrentWidget(QPainter &painter, QTransform &transform);
private:
    bool m_isAnimating;//动画正在运行
    float m_currentValue;//动画启动变化的值
    int m_currentIndex, m_previousIndex;//当前、前一个窗口索引
	int m_iWidth;//窗口宽度
	bool m_bRight;//是否向右移动
    AnimationType m_type;
    QPropertyAnimation *m_animation;//移动动画
	QPoint m_startPos;//鼠标按下坐标
	QList<QWidget*> m_lstWs;//小窗口地址
	QList<QPoint> m_lstPos;//保存小窗口坐标
	bool m_bUpdate;//是否可以进行重绘事件
	bool m_Oncepress;//每当鼠标按下时进行界面索引更新
	bool m_canMove;//鼠标按下时如果动画还未结束则鼠标移动和释放无效
	int m_savePreviousIndex;//保存鼠标移动时上一个窗口索引
};

#endif // ANIMATIONSTACKEDWIDGET_H
