#include <iostream>
#include <sstream>
#include <fstream>

#include <QMessageBox>

#include "AdvancedWidgetUI.h"
#include "NavigationDialogUI.h"
#include "WaypointsTool.h"


using namespace std;

namespace path_planner_rviz_wp_plugin
{

AdvancedWidgetUI::AdvancedWidgetUI(QWidget *parent) : QWidget(parent)
{
    ui_.setupUi(this);

    bAddedToPane_ = false;

    pWpTool_ = 0;
    pNavigationDialogUI_ = 0;

    connect(ui_.navigationPushButton, SIGNAL(pressed()), this, SLOT(OpenNavigationDialog()));
}

AdvancedWidgetUI::~AdvancedWidgetUI()
{
    std::cout << "AdvancedWidgetUI::~AdvancedWidgetUI()" << std::endl;

}

void AdvancedWidgetUI::SetWpTool(WaypointsTool* pWpTool)
{
    ROS_INFO_STREAM("AdvancedWidgetUI::SetWpTool()");
    pWpTool_ = pWpTool;
    SetConnectionsWithWpTool();
}

void AdvancedWidgetUI::SetConnectionsWithWpTool()
{
    if (pWpTool_)
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetConnections()");
        connect(ui_.playButton, SIGNAL(clicked(bool)), this, SLOT(SetPlay(bool)));
    }
}

void AdvancedWidgetUI::SetPlay(bool val)
{
    if(!pWpTool_) return; 
    
    if (val)
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetPlay() - patrolling restart");
        pWpTool_->patrollingTaskRestart();
        pWpTool_->explorationTaskRestart();
        pWpTool_->markerAddRobotTextMsg("Patrolling/Exploration Restart");
    }
    else
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetPlay() - patrolling pause");
        pWpTool_->patrollingTaskPause();
        pWpTool_->explorationTaskPause();
        pWpTool_->markerAddRobotTextMsg("Patrolling/Exploration Pause");
    }
}


void AdvancedWidgetUI::OpenNavigationDialog()
{
    if(!pNavigationDialogUI_)
    {
        pNavigationDialogUI_ = new NavigationDialogUI(this);
        if(pWpTool_)    pNavigationDialogUI_->SetWpTool(pWpTool_);
        pNavigationDialogUI_->show();
    }
    else
    {
        if (pNavigationDialogUI_->isVisible())
        {
            pNavigationDialogUI_->close();
        }
        else
        {
            pNavigationDialogUI_->show();
        }
    }
}

}