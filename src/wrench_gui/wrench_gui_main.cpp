#include <cartesio_gui/wrench_gui/wrench_gui_widget.h>

#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wrench_gui_main");

    ros::NodeHandle nhpr("~");

    QApplication a(argc, argv);
    cartesio_gui::WrenchGuiWidget main_view;
    main_view.show();
    return a.exec();
}
