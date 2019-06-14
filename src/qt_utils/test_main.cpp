#include <cartesio_gui/qt_utils/qt_utils.h>

#include <QApplication>

using namespace cartesio_gui::qt_utils;

class BoxedSliderWidgetTest: public BoxedSliderWidget
{
public:
    typedef std::shared_ptr<BoxedSliderWidgetTest> Ptr;

    BoxedSliderWidgetTest(const int id, QWidget* parent_widget = 0):
        BoxedSliderWidget(parent_widget),
        _id(id)
    {
        setLable(QString::number(id));
    }

    void on_value_changed_call_back(const int value)
    {
        std::cout<<"id: "<<_id<<" value: "<<value<<std::endl;
    }

private:
    int _id;
};

class ContainerWidget: public QWidget
{
public:
    typedef std::shared_ptr<ContainerWidget> Ptr;

    ContainerWidget(const int number_of_sliders, QWidget* parent_widget = 0):
        QWidget (parent_widget)
    {
        auto * layout = new QVBoxLayout;
        for(unsigned int i = 0; i < number_of_sliders; ++i)
            layout->addWidget(new BoxedSliderWidgetTest(i, this));

        setLayout(layout);
    }

};




int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    ContainerWidget::Ptr main_view =
            std::make_shared<ContainerWidget>(10);

    main_view->show();
    return a.exec();
}
