#include <iostream>
#include <sstream>
#include <fstream>

#include <QMessageBox>

#include "PriorityDialogUI.h"


using namespace std;

namespace path_planner_rviz_wp_plugin
{

PriorityDialogUI::PriorityDialogUI(QWidget *parent): QDialog(parent)
{
    ui_.setupUi(this);
    
    x = y = z = 0; 

    boxSliderConnector_.Init(ui_.doubleSpinBox, ui_.horizontalSlider);

    connect(&boxSliderConnector_, SIGNAL(ValueChanged(double)), this, SIGNAL(ValueChanged(double)));
}

PriorityDialogUI::~PriorityDialogUI()
{
    std::cout << "PriorityDialogUI::~PriorityDialogUI()" << std::endl;

}

void PriorityDialogUI::SetPriorityValue(double val)
{
    boxSliderConnector_.SetValue(val);
}


}