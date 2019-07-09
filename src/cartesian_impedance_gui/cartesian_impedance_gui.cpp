#include <cartesio_gui/cartesian_impedance_gui/cartesian_impedance_gui_widget.h>

using namespace cartesio_gui;

CartesianImpedanceWidget::CartesianImpedanceWidget(const double dT, QWidget* parent_widget):
    QWidget (parent_widget)
{
    /* Create GUI layout */
    auto * ui = LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);



    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("X");
    _stiffness_sliders.back().get()->setLimits(0, 300);
    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("Y");
    _stiffness_sliders.back().get()->setLimits(0, 300);
    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("Z");
    _stiffness_sliders.back().get()->setLimits(0, 300);
    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("R");
    _stiffness_sliders.back().get()->setLimits(0, 300);
    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("P");
    _stiffness_sliders.back().get()->setLimits(0, 300);
    _stiffness_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _stiffness_sliders.back().get()->setLable("Y");
    _stiffness_sliders.back().get()->setLimits(0, 300);

    auto layout_tmp = findChild<QVBoxLayout *>("verticalLayout_3");
    for(unsigned int i = 0; i < _stiffness_sliders.size(); ++i)
        layout_tmp->addWidget(_stiffness_sliders[i].get());

    _reset_stiffness = new QPushButton("Reset", parent_widget);
    connect(_reset_stiffness, &QPushButton::clicked, this, &CartesianImpedanceWidget::on_reset_stiffness_clicked);
    layout_tmp->addWidget(_reset_stiffness);

    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("X");
    _damping_sliders.back().get()->setLimits(0, 10);
    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("Y");
    _damping_sliders.back().get()->setLimits(0, 10);
    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("Z");
    _damping_sliders.back().get()->setLimits(0, 10);
    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("R");
    _damping_sliders.back().get()->setLimits(0, 10);
    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("P");
    _damping_sliders.back().get()->setLimits(0, 10);
    _damping_sliders.push_back(std::make_shared<BoxedSliderWidget>(this));
    _damping_sliders.back().get()->setLable("Y");
    _damping_sliders.back().get()->setLimits(0, 10);

    layout_tmp = findChild<QVBoxLayout *>("verticalLayout_4");
    for(unsigned int i = 0; i < _damping_sliders.size(); ++i)
        layout_tmp->addWidget(_damping_sliders[i].get());

    _reset_damping = new QPushButton("Reset", parent_widget);
    connect(_reset_damping, &QPushButton::clicked, this, &CartesianImpedanceWidget::on_reset_damping_clicked);
    layout_tmp->addWidget(_reset_damping);


    layout_tmp = findChild<QVBoxLayout *>("verticalLayout");

    _publisher_widget = std::make_shared<RosPublisherWidget<cartesian_interface::Impedance6> >(
                dT, "cartesian_interface/Impedance6");
    _actual_topic = "";

    layout_tmp->addWidget(_publisher_widget.get());

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &CartesianImpedanceWidget::update);
    _timer->start(2*dT); //ms, data are updated from the slider to the message twice the publication rate
}

void CartesianImpedanceWidget::update()
{
    if(_actual_topic != _publisher_widget->getTopic().toStdString())
    {
        _actual_topic = _publisher_widget->getTopic().toStdString();
        initSliders(_actual_topic);
    }


   for(unsigned int i = 0; i < 9; ++i)
   {
        msg.linear.stiffness[i] = 0.;
        msg.angular.stiffness[i] = 0.;
        msg.linear.damping[i] = 0.;
        msg.angular.damping[i] = 0.;
   }

   msg.linear.stiffness[0] = _stiffness_sliders[0]->getValue();
   msg.linear.stiffness[4] = _stiffness_sliders[1]->getValue();
   msg.linear.stiffness[8] = _stiffness_sliders[2]->getValue();
   msg.angular.stiffness[0] = _stiffness_sliders[3]->getValue();
   msg.angular.stiffness[4] = _stiffness_sliders[4]->getValue();
   msg.angular.stiffness[8] = _stiffness_sliders[5]->getValue();

   msg.linear.damping[0] = _damping_sliders[0]->getValue();
   msg.linear.damping[4] = _damping_sliders[1]->getValue();
   msg.linear.damping[8] = _damping_sliders[2]->getValue();
   msg.angular.damping[0] = _damping_sliders[3]->getValue();
   msg.angular.damping[4] = _damping_sliders[4]->getValue();
   msg.angular.damping[8] = _damping_sliders[5]->getValue();

    msg.header.stamp = ros::Time::now();

    _publisher_widget->fillMsg(msg);
}




//int main(int argc, char *argv[])
//{

//    ros::init(argc, argv, "cartesian_impedance_gui");

//    ros::NodeHandle nhpr("~");

//    int dT;
//    nhpr.param<int>("publish_freq", dT, 100);

//    int Kmax;
//    nhpr.param<int>("stiffness_max", Kmax, 300);

//    int Dmax;
//    nhpr.param<int>("damping_max", Dmax, 10);

//    QApplication a(argc, argv);
//    ContainerWidget::Ptr main_view =
//            std::make_shared<ContainerWidget>(dT);

//    main_view->setMaxStiffness(Kmax);
//    main_view->setMaxDamping(Dmax);



//    main_view->show();
//    return a.exec();
//}
