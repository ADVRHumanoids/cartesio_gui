#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QCoreApplication>

#include "joygui_backend.h"

int main(int argc, char *argv[])
{
    std::vector<char *> args_vec;

    for(int i = 0; i < argc; i++)
    {
        args_vec.push_back(argv[i]);
    }

    args_vec.push_back(static_cast<char *>(nullptr));

    JoyGuiBackEnd::proc_argv = args_vec.data();

    ros::init(argc, argv, "joy_gui_node");

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication::setDesktopSettingsAware(false);

    QGuiApplication app(argc, argv);

    qmlRegisterType<JoyGuiBackEnd>("joygui.backend", 1, 0, "JoyGuiBackEnd");

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}
