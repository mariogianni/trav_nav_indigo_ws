#ifndef NAVIGATION_DIALOG_UI_H
#define NAVIGATION_DIALOG_UI_H

#include <iostream>
#include <QWidget>
#include "ui_NavigationDialog.h"

#include "BoxSliderConnector.h"

namespace path_planner_rviz_wp_plugin
{

class WaypointsTool;

///	\class NavigationDialogUI
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class NavigationDialogUI: public QDialog
{
    
//https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/    
    
    Q_OBJECT
    
    
    static const double kNormalRadiusStep; 
    static const double kNormalRadiusMin;
    static const double kNormalRadiusMax;
    static const double kNormalRadiusDefault; 
    static const double kNormalRadiusModeNormal;
    static const double kNormalRadiusModeStairs;
    
    static const double kRssMinValueStep; 
    static const double kRssMinValueMin;
    static const double kRssMinValueMax;
    static const double kRssMinValueDefault;     
    
    static const double kDynamicReconfigureTimeout;
    

public:
    
    NavigationDialogUI(QWidget *parent = 0);

    ~NavigationDialogUI();

public Q_SLOTS:

    void SetNormalRadiusValue(double val);
 
    void SendNormalRadiusValue(double val);
    
    void SendNormalRadiusNormalMode();    
    void SendNormalRadiusStairsMode();
    
    void SetClosestObstVelReductionEnable();    
    void SetClosestObstVelReductionDisable();  
    
    void SetRssEnable();    
    void SetRssDisable();      
    
    void RssLoad();
    void RssSave();
           
    void SetMinRssValueEnable(double val);
    
Q_SIGNALS:

    double ValueChanged(double);


public: /// < getters
    
    
public: /// < setters    
    
    void SetRadiusValue(double val) { boxSliderNormalRadiusConnector_.SetValue(val); }    
    
    void SetWpTool(WaypointsTool* pWpTool);    
    
    void SetConnectionsWithWpTool();    

protected:

    Ui::NavigationDialog ui_;

    WaypointsTool* pWpTool_;

protected:

    BoxSliderConnector boxSliderNormalRadiusConnector_;
    BoxSliderConnector boxSliderMinRssValueConnector_;
    
};


} // namespace 

#endif