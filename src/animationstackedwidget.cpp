#include "../include/cyrobot_monitor/animationstackedwidget.h"

#include <QPixmap>
#include <QTransform>
#include <QDebug>

AnimationStackedWidget::AnimationStackedWidget(QWidget *parent)
	: QStackedWidget(parent)
{
	m_isAnimating = false;
	m_currentValue = 0;
	m_currentIndex = 0;
	m_previousIndex = 0;
	m_savePreviousIndex = 0;
	m_animation = new QPropertyAnimation(this, QByteArray());
	m_animation->setDuration(1000);
	m_animation->setEasingCurve(QEasingCurve::OutExpo);
	m_animation->setStartValue(0);
	m_animation->setEndValue(0);
	connect(m_animation, SIGNAL(valueChanged(QVariant)), SLOT(valueChanged(QVariant)));
	connect(m_animation, SIGNAL(finished()), SLOT(animationFinished()));
}
void AnimationStackedWidget::mousePressEvent(QMouseEvent *event)
{
	if (m_isAnimating)
	{
		m_bUpdate = true;
		m_canMove = false;
		return;
	}
	m_canMove = true;
	m_bUpdate = false;
	m_Oncepress = true;
	m_startPos = event->pos();

}
void AnimationStackedWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (m_isAnimating ||!m_canMove)
	{
		return;
	}
	m_bUpdate = true;
	QPoint point = event->pos() - m_startPos;
	if (m_Oncepress && abs(point.x()) >= 1)
	{
		m_Oncepress = false;
		m_savePreviousIndex = m_previousIndex;
		if (point.x() < 0)
		{
			m_bRight = false;

			m_previousIndex = m_currentIndex;
			if (m_currentIndex == 2)
			{
				m_currentIndex = 0;
			}
			else
			{
				m_currentIndex = m_currentIndex + 1;

			}

		}
		else if(point.x() > 0)
		{
			m_bRight = true;
			m_previousIndex = m_currentIndex;
			if (m_currentIndex == 0)
			{
				m_currentIndex = 2;
			}
			else
			{
				m_currentIndex = m_currentIndex - 1;
			}
			

		}
	}
	if (m_bRight)
	{
		//向右移动过程中突然又向左移动
		if (point.x() < 0)
		{
			//还原索引
			m_currentValue = 0;
			m_currentIndex = m_previousIndex;
			m_previousIndex = m_savePreviousIndex;

			//重新计算索引
			m_bRight = false;
			m_savePreviousIndex = m_previousIndex;
			m_previousIndex = m_currentIndex;
			if (m_currentIndex == 2)
			{
				m_currentIndex = 0;
			}
			else
			{
				m_currentIndex = m_currentIndex + 1;
			}
		}
		else
		{
			m_currentValue = -m_iWidth / 2 + point.x();
		}
	}

	if (!m_bRight)
	{
		//向左移动过程中突然又向右移动

		if (point.x() > 0)
		{
			//还原索引
			m_currentValue = 0;
			m_currentIndex = m_previousIndex;
			m_previousIndex = m_savePreviousIndex;

			//重新计算索引
			m_bRight = true;
			m_savePreviousIndex = m_previousIndex;
			m_previousIndex = m_currentIndex;
			if (m_currentIndex == 0)
			{
				m_currentIndex = 2;
			}
			else
			{
				m_currentIndex = m_currentIndex - 1;
			}
			
		}
		else
		{
			m_currentValue = m_iWidth / 2 + m_iWidth + point.x();
		}
	}
	update();
	currentWidget()->hide();
}
void AnimationStackedWidget::mouseReleaseEvent(QMouseEvent *event)
{

	if (m_isAnimating || !m_canMove)
	{
		return;
	}
	QPoint ptDistance = event->pos() - m_startPos;
	//向右滑
	if (abs(ptDistance.x()) >= width() / 4 && ptDistance.x() > 0)
	{
		m_bRight = true;

		m_currentValue = -m_iWidth / 2 + ptDistance.x();
		m_animation->setStartValue(-m_iWidth / 2 + ptDistance.x());
		m_animation->setEndValue(m_iWidth / 2);

		int offsetx = frameRect().width();
		int offsety = frameRect().height();
		widget(m_currentIndex)->setGeometry(0, 0, offsetx, offsety);

		currentWidget()->hide();
		m_isAnimating = true;
		m_animation->start();
	}

	//向左滑
	else  if (abs(ptDistance.x()) >= width() / 4 && ptDistance.x() < 0)
	{
		m_bRight = false;
		
		m_currentValue = m_iWidth / 2 + m_iWidth + ptDistance.x();
		m_animation->setStartValue(m_iWidth / 2 + m_iWidth + ptDistance.x());
		m_animation->setEndValue(m_iWidth / 2);

		int offsetx = frameRect().width();
		int offsety = frameRect().height();
		widget(m_currentIndex)->setGeometry(0, 0, offsetx, offsety);

		currentWidget()->hide();
		m_isAnimating = true;
		m_animation->start();

	}
	//如果移动距离过小
	else if (abs(ptDistance.x()) < width() / 4 && abs(ptDistance.x()) >0)
	{
		m_bUpdate = false;
		currentWidget()->show();
		m_currentValue = 0;
		m_currentIndex = m_previousIndex;
		m_previousIndex = m_savePreviousIndex;

	}
	update();
}
AnimationStackedWidget::~AnimationStackedWidget()
{
	delete m_animation;
}

void AnimationStackedWidget::paintEvent(QPaintEvent * event)
{
	if (m_bUpdate)
	{
		QPainter painter(this);
		QTransform transform;
		renderCurrentWidget(painter, transform);
		renderPreviousWidget(painter, transform);
	}
}

void AnimationStackedWidget::renderPreviousWidget(QPainter &painter, QTransform &transform)
{
	QWidget *w = widget(m_previousIndex);
	QRect rect = w->geometry();
	QPixmap pixmap(QSize(rect.width(), rect.height()));
	pixmap.fill(Qt::transparent);//用透明色填充
	w->render(&pixmap);
	Q_UNUSED(transform);
	if (m_bRight)
	{
		
		painter.drawPixmap(width() / 2, 0, pixmap);
	}
	else
	{
		painter.drawPixmap(-width() / 2 - width(), 0, pixmap);
	}
}

void AnimationStackedWidget::renderCurrentWidget(QPainter &painter, QTransform &transform)
{
	QWidget *w = widget(m_currentIndex);
	QRect rect = w->geometry();
	QPixmap pixmap(QSize(rect.width(), rect.height()));
	pixmap.fill(Qt::transparent);//用透明色填充
	w->render(&pixmap);
	transform.translate(m_currentValue, 0);
	painter.setTransform(transform);
	if (m_bRight)
	{
		painter.drawPixmap(-width() / 2, 0, pixmap);
	}
	else
	{
		painter.drawPixmap(-width() / 2, 0, pixmap);
		
	}
}

void AnimationStackedWidget::start(int index)
{
	if (m_isAnimating)
	{
		return;
	}
	m_bUpdate = true;

	
	if (m_bRight)
	{
		m_animation->setStartValue(-m_iWidth / 2);
		m_animation->setEndValue(m_iWidth / 2);
		
	}
	else
	{
		m_animation->setStartValue(m_iWidth / 2 + m_iWidth);
		m_animation->setEndValue(m_iWidth / 2);
	}
	m_previousIndex = m_currentIndex;
	m_currentIndex = index;

	int offsetx = frameRect().width();
	int offsety = frameRect().height();
	widget(m_currentIndex)->setGeometry(0, 0, offsetx, offsety);

	currentWidget()->hide();
	m_isAnimating = true;
	m_animation->start();
}


void AnimationStackedWidget::setNext(bool next)
{
	m_bRight = next;
}

void AnimationStackedWidget::setLength(int length, AnimationType type)
{
	m_animation->setStartValue(length / 2 + length);
	m_animation->setEndValue(length / 2);
	m_iWidth = length;
}


void AnimationStackedWidget::setWidgets(QList<QWidget*> lstWs)
{
	m_lstWs = lstWs;
}

void AnimationStackedWidget::setPos(QList<QPoint> lstPos)
{
	m_lstPos = lstPos;
	m_isAnimating = false;
}

void AnimationStackedWidget::valueChanged(const QVariant &value)
{
	m_currentValue = value.toFloat();
	update();
}

void AnimationStackedWidget::animationFinished()
{
	m_currentValue = 0;
	m_bUpdate = false;
 	QWidget *w = widget(m_currentIndex);
 	setCurrentWidget(w);
	emit sigGetWidgets(m_currentIndex);
	update();
}
