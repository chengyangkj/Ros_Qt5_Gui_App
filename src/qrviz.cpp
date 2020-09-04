#include "../include/cyrobot_monitor/qrviz.hpp"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{
    nullmap.clear();

    this->layout=layout;
    this->nodename=node_name;


    //创建rviz容器
    render_panel_=new rviz::RenderPanel;
    //向layout添加widget
    layout->addWidget(render_panel_);
    //初始化rviz控制对象
    manager_=new rviz::VisualizationManager(render_panel_);
    ROS_ASSERT(manager_ != nullptr);
    //获取当前rviz控制对象的 tool控制对象
    tool_manager_=manager_->getToolManager();
    ROS_ASSERT(tool_manager_ != nullptr);
   //初始化camera 这行代码实现放大 缩小 平移等操作
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    tool_manager_->initialize();
    manager_->removeAllDisplays();

}

QRviz::~QRviz()
{
    if (layout != nullptr && render_panel_ != nullptr)
    {
        layout->removeWidget(render_panel_);
    }
  
    if (render_panel_ != nullptr) delete render_panel_;
    if (manager_ != nullptr) delete manager_;
    
    if (current_tool != nullptr) current_tool = nullptr;
    if (tool_manager_ != nullptr) tool_manager_ = nullptr;
    
    ROS_INFO("RVIZ is shutdown");
}

///
/// \brief 获取Rviz Display树状图
///
void QRviz::GetDisplayTreeModel()
{
    rviz::PropertyTreeModel *rvizmodel = manager_->getDisplayTreeModel();
    QAbstractItemModel *model = rvizmodel;
    emit ReturnModelSignal(model);
}

///
/// \brief Rviz Display的初始化与设置
/// \param ClassID
/// \param namevalue
///
void QRviz::DisplayInit(QString ClassID, bool enabled, QMap<QString, QVariant> namevalue)
{
    int num = GetDisplayNum(ClassID);
    if (num == -1)
    {
        rviz::Display *rvizDisplay = manager_->createDisplay(ClassID, ClassID, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        rvizDisplays_.append(rvizDisplay);
        num = rvizDisplays_.indexOf(rvizDisplay);
    }
    if (!namevalue.empty())
    {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++)
        {
            rvizDisplays_[num]->subProp(it.key())->setValue(it.value());
        }
    }
    rvizDisplays_[num]->setEnabled(enabled);
    manager_->startUpdate();
}
void QRviz::DisplayInit(QString ClassID, QString name, bool enabled, QMap<QString, QVariant> namevalue)
{
    int num = GetDisplayNum(ClassID, name);
    if (num == -1)
    {
        rviz::Display *rvizDisplay = manager_->createDisplay(ClassID, name, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        rvizDisplays_.append(rvizDisplay);
        num = rvizDisplays_.indexOf(rvizDisplay);
    }
    if (!namevalue.empty())
    {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++)
        {
            rvizDisplays_[num]->subProp(it.key())->setValue(it.value());
        }
    }
    rvizDisplays_[num]->setEnabled(enabled);
    manager_->startUpdate();
}

///
/// \brief 删除Display
/// \param name
///
void QRviz::RemoveDisplay(QString name)
{
    int num = GetDisplayNumName(name);
    if (num == -1)
    {
        return ;
    }
    delete rvizDisplays_[num];
    rvizDisplays_.removeAt(num);
}
void QRviz::RemoveDisplay(QString ClassID, QString name)
{
    int num = GetDisplayNum(ClassID, name);
    if (num == -1)
    {
        return ;
    }
    delete rvizDisplays_[num];
    rvizDisplays_.removeAt(num);
}

///
/// \brief 重命名Display
/// \param oldname
/// \param newname
///
void QRviz::RenameDisplay(QString oldname, QString newname)
{
    int num = GetDisplayNumName(oldname);
    if (num == -1)
    {
        return ;
    }
    rvizDisplays_[num]->setName(newname);
}

///
/// \brief 根据Display的ClassID(和name)获得Display的序号
/// \param ClassID
/// \return 
///
int QRviz::GetDisplayNum(QString ClassID)
{
    int num = -1;
    for (int i = 0; i < rvizDisplays_.length(); i++)
    {
        if (rvizDisplays_[i] != nullptr)
        {
            if (ClassID == rvizDisplays_[i]->getClassId())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}
int QRviz::GetDisplayNum(QString ClassID, QString name)
{
    int num = -1;
    for (int i = 0; i < rvizDisplays_.length(); i++)
    {
        if (rvizDisplays_[i] != nullptr)
        {
            if (ClassID == rvizDisplays_[i]->getClassId() && name == rvizDisplays_[i]->getName())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}
int QRviz::GetDisplayNumName(QString name)
{
    int num = -1;
    for (int i = 0; i < rvizDisplays_.length(); i++)
    {
        if (rvizDisplays_[i] != nullptr)
        {
            if (name == rvizDisplays_[i]->getName())
            {
                num = i;
                break;
            }
        }
    }
    return num;
}

//设置全局显示
 void QRviz::SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate)
 {
     manager_->setFixedFrame(frame_name);
     manager_->setProperty("Background Color",backColor);
     manager_->setProperty("Frame Rate",frame_rate);
     manager_->startUpdate();
 }

// "rviz/MoveCamera";
// "rviz/Interact";
// "rviz/Select";
// "rviz/SetInitialPose";
// "rviz/SetGoal";
 //设置机器人导航初始位置
 void QRviz::Set_Pos()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/SetInitialPose");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();

//     tool_manager_->setCurrentTool()

 }
 //设置机器人导航目标点
 void QRviz::Set_Goal()
 {
     //添加工具
     current_tool= tool_manager_->addTool("rviz/SetGoal");
     //设置goal的话题
     rviz::Property* pro= current_tool->getPropertyContainer();
     pro->subProp("Topic")->setValue("/move_base_simple/goal");
     //设置当前frame
     manager_->setFixedFrame("map");
     //设置当前使用的工具为SetGoal（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );

     manager_->startUpdate();

 }
 void QRviz::Set_MoveCamera()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/MoveCamera");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();
 }
 void QRviz::Set_Select()
 {
     //获取设置Pos的工具
     //添加工具

     current_tool= tool_manager_->addTool("rviz/Select");
     //设置当前使用的工具为SetInitialPose（实现在地图上标点）
     tool_manager_->setCurrentTool( current_tool );
     manager_->startUpdate();
 }

 void QRviz::addTool( rviz::Tool* )
 {

 }
void QRviz::createDisplay(QString display_name,QString topic_name)
{
    
}
void QRviz::run()
{

}
