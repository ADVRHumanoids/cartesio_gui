#include <cartesio_gui/wrench_gui/wrench_gui_widget.h>

#include <QApplication>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wrench_gui_main");

    ros::NodeHandle nhpr("~");

    QApplication a(argc, argv);
    cartesio_gui::WrenchGuiWidget::Ptr main_view =
            std::make_shared<cartesio_gui::WrenchGuiWidget>(100);

    //main_view->setLimits(-200, 200);

    main_view->show();
    return a.exec();
}
