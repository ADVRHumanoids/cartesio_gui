#ifndef JOYGUI_BACKEND_H
#define JOYGUI_BACKEND_H

#include <QObject>
#include <QVector>

#include <cartesian_interface/ros/RosImpl.h>
#include <cartesian_interface/JoystickStatus.h>

class JoyGuiBackEnd : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString activeTask READ activeTask WRITE setActiveTask  NOTIFY activeTaskChanged)

public:

    explicit JoyGuiBackEnd(QObject * parent = nullptr);

    Q_INVOKABLE QStringList getTasks();
    Q_INVOKABLE QStringList getLinks();

    QString activeTask() const;
    void setActiveTask(QString active_task);
    Q_INVOKABLE int activeTaskIndex() const;

    Q_INVOKABLE QVector<int> getEnabledAxis() const;
    Q_INVOKABLE void setEnabledAxis(int idx,  bool enabled);

    Q_INVOKABLE qreal getMaxLinearSpeed() const;
    Q_INVOKABLE qreal getMaxAngularSpeed() const;
    Q_INVOKABLE void setMaxLinearSpeed(qreal vel);
    Q_INVOKABLE void setMaxAngularSpeed(qreal vel);

    Q_INVOKABLE QString getRefFrame() const;
    Q_INVOKABLE void setRefFrame(QString ref_frame);

    Q_INVOKABLE void enableVelocityControl(bool enabled);

    Q_INVOKABLE QString getBaseLink() const;
    Q_INVOKABLE void setBaseLink(QString base_link);

    enum class ControlMode
    {
        Position, Velocity, Disabled
    };

    Q_ENUM(ControlMode)
    Q_INVOKABLE ControlMode getControlMode(QString task) const;

    Q_INVOKABLE QVector<qreal> getTwist();


signals:

    void activeTaskChanged();
    void joyStatusReceived();

public slots:



private:

    struct JoyData
    {
        ros::Subscriber twist_sub;
        QString ref_frame;

        JoyData();
    };

    typedef Eigen::Matrix<int, 6, 1> Vector6i;

    void on_twist_recv(const geometry_msgs::TwistStampedConstPtr& msg, std::string task_id);
    void on_joy_status_recv(const cartesian_interface::JoystickStatusConstPtr& msg);

    XBot::Cartesian::RosImpl _ci;
    ros::NodeHandle _nh;

    ros::Subscriber _status_sub;

    std::map<std::string, JoyData> _joy_map;
    Vector6i _enabled_axis;
    double _max_linear, _max_angular;

    QString _active_task;

    ros::Time _twist_recv_stamp;
    QVector<qreal> _twist;
};

#endif // JOYGUI_BACKEND_H
