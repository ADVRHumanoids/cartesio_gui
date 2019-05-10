#include "joygui_backend.h"
#include <urdf_parser/urdf_parser.h>
#include <cartesian_interface/JoystickStatus.h>
#include <cartesian_interface/SetJoystickTaskBaseFrame.h>
#include <cartesian_interface/SetJoystickActiveTask.h>
#include <cartesian_interface/SetJoystickTaskMaxSpeed.h>
#include <cartesian_interface/ToggleAxis.h>

JoyGuiBackEnd::JoyGuiBackEnd(QObject *parent) :
    QObject(parent),
    _nh("cartesian")
{
    auto tasks = _ci.getTaskList();
    for(auto t : tasks)
    {
        auto sub = _nh.subscribe<geometry_msgs::TwistStamped>(t + "/velocity_reference",
                                                              1,
                                                              boost::bind(&JoyGuiBackEnd::on_twist_recv, this, _1, t));
        JoyData jd;
        jd.twist_sub = sub;

        _joy_map[t] = jd;
    }

    _status_sub = _nh.subscribe<cartesian_interface::JoystickStatus>("joystick/joystick_status",
                                                                     1,
                                                                     &JoyGuiBackEnd::on_joy_status_recv, this);

    setActiveTask(QString::fromStdString(tasks[1]));

    _twist.fill(0, 6);
    _enabled_axis.setOnes();
}

QStringList JoyGuiBackEnd::getTasks()
{
    auto tasks = _ci.getTaskList();
    QStringList q_tasks;

    for(auto t : tasks)
    {
        q_tasks.push_back(QString::fromStdString(t));
    }

    return q_tasks;
}

QStringList JoyGuiBackEnd::getLinks()
{
    std::string urdf;
    
    ros::NodeHandle nh_base;
    if(nh_base.hasParam("model_description/robot_description"))
    {
        nh_base.getParam("model_description/robot_description", urdf);
    }
    else if(nh_base.hasParam("robot_description"))
    {
        nh_base.getParam("robot_description", urdf);
    }
    else
    {
        throw std::runtime_error("Unable to get 'robot_description' parameter");
    }
    
    auto urdfdom = urdf::parseURDF(urdf);

    std::vector<urdf::LinkSharedPtr> link_vec;
    urdfdom->getLinks(link_vec);

    QStringList q_links;

    for(auto l : link_vec)
    {
        q_links.push_back(QString::fromStdString(l->name));
    }

    return q_links;
}

QString JoyGuiBackEnd::activeTask() const
{
    return _active_task;
}

void JoyGuiBackEnd::setActiveTask(QString active_task)
{
    _active_task = active_task;
    _twist.fill(0, 6);

    cartesian_interface::SetJoystickActiveTask srv;
    srv.request.active_task = active_task.toStdString();
    if(!ros::service::call("cartesian/joystick/set_active_task", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_active_task'" << std::endl;
    }

    emit activeTaskChanged();
}

int JoyGuiBackEnd::activeTaskIndex() const
{
    auto tasks = _ci.getTaskList();
    auto it = std::find(tasks.begin(), tasks.end(), _active_task.toStdString());

    return it - tasks.begin();
}

QVector<int> JoyGuiBackEnd::getEnabledAxis() const
{
    QVector<int> q_enabled_axis;
    q_enabled_axis.resize(6);
    q_enabled_axis[0] = _enabled_axis[0];
    q_enabled_axis[1] = _enabled_axis[1];
    q_enabled_axis[2] = _enabled_axis[2];
    q_enabled_axis[3] = _enabled_axis[3];
    q_enabled_axis[4] = _enabled_axis[4];
    q_enabled_axis[5] = _enabled_axis[5];

    return q_enabled_axis;
}

void JoyGuiBackEnd::setEnabledAxis(int idx, bool enabled)
{
    _enabled_axis[idx] = enabled;

    cartesian_interface::ToggleAxis srv;
    srv.request.axis_mask[0] = _enabled_axis[0];
    srv.request.axis_mask[1] = _enabled_axis[1];
    srv.request.axis_mask[2] = _enabled_axis[2];
    srv.request.axis_mask[3] = _enabled_axis[3];
    srv.request.axis_mask[4] = _enabled_axis[4];
    srv.request.axis_mask[5] = _enabled_axis[5];

    if(!ros::service::call("cartesian/joystick/toggle_axis", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/toggle_axis'" << std::endl;
    }

    std::cout << srv.response.message << std::endl;

}

qreal JoyGuiBackEnd::getMaxLinearSpeed() const
{
    return _joy_map.at(_active_task.toStdString()).max_linear;
}

qreal JoyGuiBackEnd::getMaxAngularSpeed() const
{
    return _joy_map.at(_active_task.toStdString()).max_angular;
}

void JoyGuiBackEnd::setMaxLinearSpeed(qreal vel)
{
    _joy_map.at(_active_task.toStdString()).max_linear = vel;

    cartesian_interface::SetJoystickTaskMaxSpeed srv;
    srv.request.max_linear_speed = vel;
    srv.request.max_angular_speed = getMaxAngularSpeed();

    if(!ros::service::call("cartesian/joystick/set_max_speed", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_max_speed'" << std::endl;
    }
}

void JoyGuiBackEnd::setMaxAngularSpeed(qreal vel)
{
    _joy_map.at(_active_task.toStdString()).max_angular = vel;

    cartesian_interface::SetJoystickTaskMaxSpeed srv;
    srv.request.max_linear_speed = getMaxLinearSpeed();
    srv.request.max_angular_speed = vel;

    if(!ros::service::call("cartesian/joystick/set_max_speed", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_max_speed'" << std::endl;
    }
}

QString JoyGuiBackEnd::getRefFrame() const
{
    return _joy_map.at(_active_task.toStdString()).ref_frame;
}

void JoyGuiBackEnd::setRefFrame(QString ref_frame)
{
    cartesian_interface::SetJoystickTaskBaseFrame srv;
    srv.request.base_frame = ref_frame.toStdString();
    if(!ros::service::call("cartesian/joystick/set_base_frame", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_base_frame'" << std::endl;
    }
}

void JoyGuiBackEnd::enableVelocityControl(bool enabled) try
{
    using CtrlMode = XBot::Cartesian::ControlType;
    _ci.setControlMode(_active_task.toStdString(), enabled ? CtrlMode::Velocity : CtrlMode::Position);
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

QString JoyGuiBackEnd::getBaseLink() const try
{
    return QString::fromStdString(_ci.getBaseLink(_active_task.toStdString()));
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

void JoyGuiBackEnd::setBaseLink(QString base_link) try
{
    _ci.setBaseLink(_active_task.toStdString(), base_link.toStdString());
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

JoyGuiBackEnd::ControlMode JoyGuiBackEnd::getControlMode(QString task) const try
{
    auto ci_ctrl = _ci.getControlMode(task.toStdString());

    switch(ci_ctrl)
    {
        case XBot::Cartesian::ControlType::Position:
            return ControlMode::Position;

        case XBot::Cartesian::ControlType::Velocity:
            return ControlMode::Velocity;

        case XBot::Cartesian::ControlType::Disabled:
            return ControlMode::Disabled;
    }
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

QVector<qreal> JoyGuiBackEnd::getTwist()
{
    ros::spinOnce();

    if((ros::Time::now() - _twist_recv_stamp).toSec() > 0.2)
    {
        _twist.fill(0, 6);
    }

    return _twist;
}

void JoyGuiBackEnd::on_twist_recv(const geometry_msgs::TwistStampedConstPtr& msg, std::string task_id)
{
    if(task_id != _active_task.toStdString())
    {
        return;
    }

    _joy_map.at(_active_task.toStdString()).ref_frame = QString::fromStdString(msg->header.frame_id);

    _twist_recv_stamp = ros::Time::now();

    _twist[0] = msg->twist.linear.x;
    _twist[1] = msg->twist.linear.y;
    _twist[2] = msg->twist.linear.z;
    _twist[3] = msg->twist.angular.x;
    _twist[4] = msg->twist.angular.y;
    _twist[5] = msg->twist.angular.z;
}

void JoyGuiBackEnd::on_joy_status_recv(const cartesian_interface::JoystickStatusConstPtr & msg)
{
    if(msg->active_task != _active_task.toStdString())
    {
        setActiveTask(QString::fromStdString(msg->active_task));
    }

    _joy_map.at(_active_task.toStdString()).max_linear = msg->max_linear_speed;
    _joy_map.at(_active_task.toStdString()).max_angular = msg->max_angular_speed;

    emit joyStatusReceived();

}

JoyGuiBackEnd::JoyData::JoyData():
    max_linear(0.0),
    max_angular(0.0),
    ref_frame("")
{

}
