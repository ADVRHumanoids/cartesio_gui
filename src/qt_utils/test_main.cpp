#include <cartesio_gui/qt_utils/qt_utils.h>
#include <geometry_msgs/WrenchStamped.h>

#include <QApplication>

using namespace cartesio_gui::qt_utils;

class BoxedSliderWidgetTest: public BoxedSliderWidget
{
public:
    typedef std::shared_ptr<BoxedSliderWidgetTest> Ptr;

    BoxedSliderWidgetTest(const QString& id, QWidget* parent_widget = 0):
        BoxedSliderWidget(parent_widget),
        _id(id)
    {
        setLable(id);
    }

    void on_value_changed_call_back(const int value)
    {
        std::cout<<"id: "<<_id.toStdString()<<" value: "<<value<<std::endl;
    }

private:
    QString _id;
};

class ContainerWidget: public QWidget
{
public:
    typedef std::shared_ptr<ContainerWidget> Ptr;

    ContainerWidget(QWidget* parent_widget = 0):
        QWidget (parent_widget)
    {
        auto * layout = new QVBoxLayout;

        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("x", this));
        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("y", this));
        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("z", this));
        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("R", this));
        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("P", this));
        _sliders.push_back(std::make_shared<BoxedSliderWidgetTest>("Y", this));

        for(unsigned int i = 0; i < _sliders.size(); ++i)
            layout->addWidget(_sliders[i].get());

        _publisher_widget = std::make_shared<RosPublisherWidget<geometry_msgs::WrenchStamped> >(
                    100, "geometry_msgs/WrenchStamped");

        layout->addWidget(_publisher_widget.get());

        setLayout(layout);



        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &ContainerWidget::update);
        _timer->start(200); //ms
    }

    geometry_msgs::WrenchStamped msg;

private:
    QTimer* _timer;

    std::vector<BoxedSliderWidget::Ptr> _sliders;

    BoxedSliderWidgetTest::Ptr _selector;

    ros::NodeHandle _n;

    void update()
    {
        msg.wrench.force.x = _sliders[0]->getValue();
        msg.wrench.force.y = _sliders[1]->getValue();
        msg.wrench.force.z = _sliders[2]->getValue();

        msg.wrench.torque.x = _sliders[3]->getValue();
        msg.wrench.torque.y = _sliders[4]->getValue();
        msg.wrench.torque.z = _sliders[5]->getValue();

        msg.header.stamp = ros::Time::now();

        _publisher_widget->fillMsg(msg);
    }

    RosPublisherWidget<geometry_msgs::WrenchStamped>::Ptr _publisher_widget;

};




int main(int argc, char *argv[])
{

    ros::init(argc, argv, "test_widget");

    QApplication a(argc, argv);
    ContainerWidget::Ptr main_view =
            std::make_shared<ContainerWidget>();

    main_view->show();
    return a.exec();
}
