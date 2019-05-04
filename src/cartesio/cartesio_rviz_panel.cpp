#include "cartesio_gui/cartesio/task_options_widget.h"

#include <ros/ros.h>
#include <rviz/panel.h>



class CartesioRvizPanel : public rviz::Panel 
{
    
public:
    
    CartesioRvizPanel(QWidget * parent = nullptr);
    
private:
    
    
};

CartesioRvizPanel::CartesioRvizPanel(QWidget * parent):
    rviz::Panel(parent)
{
    
    auto * vbox = new QVBoxLayout(parent);
    
    vbox->addWidget(new cartesio_gui::TaskOptionsWidget(parent, ros::NodeHandle()));
    setLayout(vbox);
    
}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CartesioRvizPanel, rviz::Panel)
