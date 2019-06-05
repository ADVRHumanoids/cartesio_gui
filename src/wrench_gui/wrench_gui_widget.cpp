#include <cartesio_gui/wrench_gui/wrench_gui_widget.h>


using namespace cartesio_gui;

QWidget * LoadUiFile(QWidget * parent)
{
    QUiLoader loader;

    QFile file(":/ui/wrench_gui_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

WrenchGuiWidget::WrenchGuiWidget(const int dT_ms, QWidget * parent):
       QWidget(parent),
       _ros("geometry_msgs/WrenchStamped")
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    _reset_button = findChild<QPushButton *>("reset_button");
    connect(_reset_button, &QPushButton::clicked, this, &WrenchGuiWidget::on_reset_button_clicked);

    _send_button = findChild<QPushButton *>("send_button");
    _send_button->setText("Send");
    _send_reference = false;
    connect(_send_button, &QPushButton::clicked, this, &WrenchGuiWidget::on_send_button_clicked);

    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("XhorizontalSlider", "XdoubleSpinBox", this));
    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("YhorizontalSlider", "YdoubleSpinBox", this));
    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("ZhorizontalSlider", "ZdoubleSpinBox", this));
    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("RollhorizontalSlider", "RolldoubleSpinBox", this));
    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("PitchhorizontalSlider", "PitchdoubleSpinBox", this));
    _sliders.push_back(std::make_shared<SliderSpinBoxSync>
                       ("YawhorizontalSlider", "YawdoubleSpinBox", this));

    _topic_selector = findChild<QComboBox *>("topic_selector");
    fillTopicSelector(_ros.getTopics());
    connect(_topic_selector,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &WrenchGuiWidget::on_topic_changed);

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &WrenchGuiWidget::update);
    _timer->start(dT_ms); //ms

}

void WrenchGuiWidget::on_topic_changed(int i)
{
    _topic = _topic_selector->itemText(i);
    std::cout<<"Publish to: "<<_topic.toStdString()<<std::endl;
}

void WrenchGuiWidget::fillTopicSelector(const std::vector<std::string>& topics)
{
    if(topics.size() > 0)
    {
        for(unsigned int i = 0; i < topics.size(); ++i)
        {
            QString s(topics[i].c_str());
            _available_topics.push_back(s);
        }

        for(unsigned int i = 0; i < _available_topics.size(); ++i)
            _topic_selector->insertItem(i, _available_topics[i]);
    }
    else {
        _topic_selector->insertItem(0, "none");
    }
}

void WrenchGuiWidget::on_reset_button_clicked(bool checked)
{
    for(unsigned int i = 0; i < _sliders.size(); ++i)
        _sliders[i]->setValue(0);
}

void WrenchGuiWidget::on_send_button_clicked(bool checked)
{
    if(_send_reference)
    {
        _send_reference = false;
        _send_button->setText("Send");
        std::cout<<"STOP sending references"<<std::endl;
    }
    else
    {
        _send_reference = true;
        _send_button->setText("Stop");
        std::cout<<"START sending references"<<std::endl;
    }

}

void WrenchGuiWidget::update()
{
    if(_send_reference && _ros.getTopics().size() > 0)
    {
        geometry_msgs::WrenchStamped msg;
        msg.header.stamp = ros::Time::now();

        msg.wrench.force.x = _sliders[0]->getValue();
        msg.wrench.force.y = _sliders[1]->getValue();
        msg.wrench.force.z = _sliders[2]->getValue();

        msg.wrench.torque.x = _sliders[3]->getValue();
        msg.wrench.torque.y = _sliders[4]->getValue();
        msg.wrench.torque.z = _sliders[5]->getValue();

        _ros.publish(_topic.toStdString(), msg);
    }
}

void WrenchGuiWidget::setLimits(const int min, const int max)
{
    for(unsigned int i = 0; i < _sliders.size(); ++i)
        _sliders[i]->setLimits(min, max);
}
