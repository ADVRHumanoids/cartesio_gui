#include <pluginlib/class_list_macros.h>
#include <cartesio_gui/cartesian_impedance_gui/cartesian_impedance_gui_widget.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

namespace cartesio_rqt
{

class CartesianImpedanceWidgetRqt : public rqt_gui_cpp::Plugin
{
//     Q_OBJECT
    
public:
    
    CartesianImpedanceWidgetRqt();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    
    virtual void shutdownPlugin(){}
    
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const {}
                              
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings) {}
    
    
private:
    
    QWidget* widget_;

    
};


CartesianImpedanceWidgetRqt::CartesianImpedanceWidgetRqt()
{
    setObjectName("XBotCartesianImpedanceWidget");
}

void CartesianImpedanceWidgetRqt::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    
    const double dT = 100.;
    const double K_max = 1000.;
    const double D_max = 30.;
    
    auto wrench_gui = new cartesio_gui::ContainerWidget(dT);
    wrench_gui->setMaxStiffness(K_max);
    wrench_gui->setMaxDamping(D_max);
    widget_ = wrench_gui;
    widget_->setWindowTitle("Cartesian Impedance Sliders");
    
    // add widget to the user interface
    context.addWidget(widget_);
}

}


PLUGINLIB_EXPORT_CLASS(cartesio_rqt::CartesianImpedanceWidgetRqt, rqt_gui_cpp::Plugin)
