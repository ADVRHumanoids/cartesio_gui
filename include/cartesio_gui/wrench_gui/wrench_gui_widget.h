#ifndef WRENCHGUIWIDGET_H
#define WRENCHGUIWIDGET_H

#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <functional>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>


namespace cartesio_gui
{

template <class data_type>
class RosInterface
{
public:
    RosInterface(const std::string& interested_data_type):
        _data_type(interested_data_type)
    {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
          const ros::master::TopicInfo& info = *it;
          if(info.datatype == _data_type)
              _topics.push_back(info.name);
        }

        std::cout<<"Available topics:"<<std::endl;
        for(unsigned int i = 0; i < _topics.size(); ++i)
        {
            std::cout<<_topics[i]<<std::endl;

            _publishers[_topics[i]] = _n.advertise<data_type>(_topics[i], 1);
        }
    }

    std::vector<std::string> getTopics(){return _topics;}

    void publish(const std::string& topic, const data_type& msg)
    {
        _publishers[topic].publish(msg);
    }
private:
    std::string _data_type;
    std::vector<std::string> _topics;
    ros::NodeHandle _n;
    std::map<std::string, ros::Publisher> _publishers;

};

class SliderSpinBoxSync: public QObject
{
public:
    typedef std::shared_ptr<SliderSpinBoxSync> Ptr;

    SliderSpinBoxSync(const QString& slider_name,
                      const QString& spin_box_name,
                      QWidget* parent_widget):
        _parent_widget(parent_widget)
    {
        _slider = parent_widget->findChild<QSlider *>(slider_name);
        _spin_box = parent_widget->findChild<QDoubleSpinBox *>(spin_box_name);

        this->setLimits(-100, 100);

        connect(_slider, &QSlider::valueChanged,
                    this, &SliderSpinBoxSync::on_slider_value_changed);

        connect(_spin_box, &QDoubleSpinBox::editingFinished,
                    this, &SliderSpinBoxSync::on_box_value_changed);
    }

    double getValue(){return _value;}

    void on_slider_value_changed(int value)
    {
        _value = value;
        _spin_box->setValue(_value);
    }

    void on_box_value_changed()
    {
        _value = _spin_box->value();
        _slider->setValue(_value);
    }

    void setLimits(int min, int max)
    {
        _min = min;
        _max = max;

        _slider->setMinimum(_min);
        _slider->setMaximum(_max);

        _spin_box->setMinimum(_min);
        _spin_box->setMaximum(_max);
    }

    void setValue(int value)
    {
        _value = value;
        _spin_box->setValue(_value);
        _slider->setValue(_value);
    }

private:
    QSlider* _slider;
    QDoubleSpinBox* _spin_box;
    QWidget* _parent_widget;

    double _value;

    int _min, _max;



};

class WrenchGuiWidget : public QWidget
{

public:
    typedef std::shared_ptr<WrenchGuiWidget> Ptr;

    WrenchGuiWidget(const int dT_ms, QWidget * parent = 0);
    void update();

private:
    QPushButton* _reset_button;
    void on_reset_button_clicked(bool checked);

    QPushButton* _send_button;
    bool _send_reference;
    void on_send_button_clicked(bool checked);

    std::vector<SliderSpinBoxSync::Ptr> _sliders;

    QComboBox* _topic_selector;
    QStringList _available_topics;
    void fillTopicSelector(const std::vector<std::string>& topics);
    void on_topic_changed(int i);

    QString _topic;
    RosInterface<geometry_msgs::WrenchStamped> _ros;

    QTimer* _timer;

};

}

#endif
