#ifndef ADVANCED_WIDGET_UI_H
#define ADVANCED_WIDGET_UI_H

#include <iostream>
#include <QWidget>
#include "ui_AdvancedWidget.h"


namespace path_planner_rviz_wp_plugin
{

class NavigationDialogUI;

class WaypointsTool;

///	\class AdvancedWidgetUI
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning

class AdvancedWidgetUI : public QWidget
{
    Q_OBJECT

public:
    AdvancedWidgetUI(QWidget *parent = 0);

    ~AdvancedWidgetUI();
    
public: /// < getters

    bool IsAddedToPane() const {return bAddedToPane_;}
    
public: /// < setters     
    
    void SetAddedToPane(bool val) {bAddedToPane_ = val;}
    
    void SetWpTool(WaypointsTool* pWpTool);
    
    void SetConnectionsWithWpTool();
    
public Q_SLOTS:

    void SetPlay(bool val);    
    
    void OpenNavigationDialog();    
    
protected:

    Ui::AdvancedWidget ui_;


protected:

    bool bAddedToPane_;

    WaypointsTool* pWpTool_;
    
    NavigationDialogUI* pNavigationDialogUI_;
};


} // namespace 

#endif