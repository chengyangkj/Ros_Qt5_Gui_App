#ifndef MAINFORM_H
#define MAINFORM_H

#include <QWidget>
#include <QDebug>
#include "qnode.hpp"
namespace Ui {
class MainForm;
}

class MainForm : public QWidget
{
    Q_OBJECT

public:
    explicit MainForm(QWidget *parent = nullptr);
    ~MainForm();
    void closeEvent(QCloseEvent *event); // Overloaded function
private:
    Ui::MainForm *ui;
    QObject* qmlRoot;
    cyrobot_monitor_simple::QNode* qnode;
};

#endif // MAINFORM_H
