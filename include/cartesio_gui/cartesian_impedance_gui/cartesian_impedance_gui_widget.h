#ifndef __CARTESIAN_IMPEDANCE_GUI_WIDGET_H__
#define __CARTESIAN_IMPEDANCE_GUI_WIDGET_H__

#include <ros/ros.h>
#include <cartesio_gui/qt_utils/qt_utils.h>
#include <QApplication>
#include <cartesian_interface/Impedance6.h>

using namespace cartesio_gui::qt_utils;

namespace cartesio_gui{
class ContainerWidget: public QWidget
{
public:
    typedef std::shared_ptr<ContainerWidget> Ptr;

    ContainerWidget(const double dT, QWidget* parent_widget = 0);

    cartesian_interface::Impedance6 msg;

    void setMaxStiffness(const int stiffness_max)
    {
        for(unsigned int i = 0; i < _stiffness_sliders.size(); ++i)
            _stiffness_sliders[i]->setLimits(0, stiffness_max);
    }

    void setMaxDamping(const int damping_max)
    {
        for(unsigned int i = 0; i < _damping_sliders.size(); ++i)
            _damping_sliders[i]->setLimits(0, damping_max);
    }



private:
    QTimer* _timer;

    std::vector<BoxedSliderWidget::Ptr> _stiffness_sliders;
    std::vector<BoxedSliderWidget::Ptr> _damping_sliders;

    BoxedSliderWidget::Ptr _selector;

    ros::NodeHandle _n;

    void update();

    QWidget * LoadUiFile(QWidget * parent)
    {
        QUiLoader loader;

        QFile file(":/ui/cartesian_impedance_gui.ui");
        file.open(QFile::ReadOnly);

        QWidget *formWidget = loader.load(&file, parent);
        file.close();

        return formWidget;
    }

    RosPublisherWidget<cartesian_interface::Impedance6>::Ptr _publisher_widget;

    QPushButton* _reset_stiffness;
    QPushButton* _reset_damping;
    void on_reset_stiffness_clicked(bool checked)
    {
        for(unsigned int i = 0; i < _stiffness_sliders.size(); ++i)
            _stiffness_sliders[i]->setValue(0.);
    }
    void on_reset_damping_clicked(bool checked)
    {
        for(unsigned int i = 0; i < _damping_sliders.size(); ++i)
            _damping_sliders[i]->setValue(0.);
    }

};
}

#endif
