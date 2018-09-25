#ifndef PRIORITY_DIALOG_UI_H
#define PRIORITY_DIALOG_UI_H

#include <iostream>
#include <QWidget>
#include "ui_PriorityDialog.h"

#include "BoxSliderConnector.h"

namespace path_planner_rviz_wp_plugin
{


///	\class PriorityDialogUI
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class PriorityDialogUI: public QDialog
{
    Q_OBJECT

public:
    PriorityDialogUI(QWidget *parent = 0);

    ~PriorityDialogUI();

public Q_SLOTS:

    void SetPriorityValue(double val);

    void SetXYZ(double x_in, double y_in, double z_in)
    {
        x = x_in;
        y = y_in;
        z = z_in; 
    }
    
Q_SIGNALS:

    double ValueChanged(double);


public: /// < getters
    
    std::string GetMarkerName() { return markerName_; }
    
    double GetX() const {return x;}
    double GetY() const {return y;}
    double GetZ() const {return z;}
    
public: /// < setters    
    
    void SetMarkerName(const std::string& markerName) { markerName_ = markerName; }
    void SetValue(double val) { boxSliderConnector_.SetValue(val); }    
        
protected:

    Ui::PriorityDialog ui_;


protected:

    BoxSliderConnector boxSliderConnector_;
    std::string markerName_; 
    double x, y, z; 
};


} // namespace 

#endif