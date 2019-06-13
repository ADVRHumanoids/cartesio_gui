#include <cartesio_gui/wrench_gui/wrench_gui_widget.h>

#include <QApplication>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wrench_gui_main");

    ros::NodeHandle nhpr("~");

    int dT;
    nhpr.param<int>("publish_freq", dT, 100);

    int lims;
    nhpr.param<int>("limits", lims, 100);


    QApplication a(argc, argv);
    cartesio_gui::WrenchGuiWidget::Ptr main_view =
            std::make_shared<cartesio_gui::WrenchGuiWidget>(dT);

    main_view->setLimits(-lims, lims);

    main_view->show();
    return a.exec();
}
