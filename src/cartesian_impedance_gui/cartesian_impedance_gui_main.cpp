#include <ros/ros.h>
#include <cartesio_gui/qt_utils/qt_utils.h>
#include <QApplication>
#include <cartesian_interface/Impedance6.h>

using namespace cartesio_gui::qt_utils;

class ContainerWidget: public QWidget
{
public:
    typedef std::shared_ptr<ContainerWidget> Ptr;

    ContainerWidget(QWidget* parent_widget = 0):
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
        connect(_reset_stiffness, &QPushButton::clicked, this, &ContainerWidget::on_reset_stiffness_clicked);
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
        connect(_reset_damping, &QPushButton::clicked, this, &ContainerWidget::on_reset_damping_clicked);
        layout_tmp->addWidget(_reset_damping);


        layout_tmp = findChild<QVBoxLayout *>("verticalLayout");

        _publisher_widget = std::make_shared<RosPublisherWidget<cartesian_interface::Impedance6> >(
                    100, "cartesian_interface/Impedance6");

        layout_tmp->addWidget(_publisher_widget.get());

        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &ContainerWidget::update);
        _timer->start(200); //ms
    }

    cartesian_interface::Impedance6 msg;

private:
    QTimer* _timer;

    std::vector<BoxedSliderWidget::Ptr> _stiffness_sliders;
    std::vector<BoxedSliderWidget::Ptr> _damping_sliders;

    BoxedSliderWidget::Ptr _selector;

    ros::NodeHandle _n;

    void update()
    {
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

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "cartesian_impedance_gui");

    QApplication a(argc, argv);
    ContainerWidget::Ptr main_view =
            std::make_shared<ContainerWidget>();

    main_view->show();
    return a.exec();
}
