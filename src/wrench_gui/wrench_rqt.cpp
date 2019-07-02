#include <pluginlib/class_list_macros.h>
#include <cartesio_gui/wrench_gui/wrench_gui_widget.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

namespace cartesio_rqt
{

class WrenchWidgetRqt : public rqt_gui_cpp::Plugin
{
//     Q_OBJECT
    
public:
    
    WrenchWidgetRqt();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    
    virtual void shutdownPlugin(){}
    
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const {}
                              
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings) {}

    // Comment in to signal that the plugin has a way to configure it
    //bool hasConfiguration() const;
    //void triggerConfiguration();
    
    
private:
    
    QWidget* widget_;

    
};


WrenchWidgetRqt::WrenchWidgetRqt()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    setObjectName("XBotWrenchSlider");
//     setWindowTitle("Joint Sliders");
    
}

void WrenchWidgetRqt::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    
    const double freq = 10.;
    const double lims = 100.;
    
    auto wrench_gui = new cartesio_gui::WrenchGuiWidget(freq);
    wrench_gui->setLimits(-lims, lims);
    widget_ = wrench_gui;
    widget_->setWindowTitle("Wrench Sliders");
    
    // add widget to the user interface
    context.addWidget(widget_);
}

}


PLUGINLIB_EXPORT_CLASS(cartesio_rqt::WrenchWidgetRqt, rqt_gui_cpp::Plugin)
