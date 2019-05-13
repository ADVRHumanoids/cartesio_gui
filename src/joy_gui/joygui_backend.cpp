#include "joygui_backend.h"
#include <urdf_parser/urdf_parser.h>
#include <cartesian_interface/JoystickStatus.h>
#include <cartesian_interface/SetJoystickTaskBaseFrame.h>
#include <cartesian_interface/SetJoystickActiveTask.h>
#include <cartesian_interface/SetJoystickTaskMaxSpeed.h>
#include <cartesian_interface/ToggleAxis.h>

char ** JoyGuiBackEnd::proc_argv = nullptr;

JoyGuiBackEnd::JoyGuiBackEnd(QObject *parent) :
    QObject(parent),
    _nh("cartesian")
{
    try
    {
        construct();
    }
    catch(std::exception& e)
    {
        std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;

        for(auto t : getTasks())
        {
            _joy_map[t.toStdString()];
        }
    }
}

void JoyGuiBackEnd::restart_process()
{
    execv("/proc/self/exe", proc_argv);
}

void JoyGuiBackEnd::construct()
{
    _joy_map.clear();

    _twist.fill(0, 6);
    _enabled_axis.setOnes();
    _active_task = "MyTask1";

    _ci = std::make_shared<XBot::Cartesian::RosImpl>();

    auto tasks = ci()->getTaskList();

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

    _active_task = QString::fromStdString(tasks[1]);


}

QStringList JoyGuiBackEnd::getTasks() try
{

    auto tasks = ci()->getTaskList();
    QStringList q_tasks;

    for(auto t : tasks)
    {
        q_tasks.push_back(QString::fromStdString(t));
    }

    return q_tasks;
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
    return {"MyTask1", "MyTask2", "MyTask3"};
}

QStringList JoyGuiBackEnd::getLinks() try
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
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
    return {"MyLink1", "MyLink2", "MyLink3"};
}


QString JoyGuiBackEnd::activeTask() const
{
    return _active_task;
}

void JoyGuiBackEnd::setActiveTask(QString active_task) try
{
    ci();

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
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

int JoyGuiBackEnd::activeTaskIndex() const try
{
    auto tasks = ci()->getTaskList();
    auto it = std::find(tasks.begin(), tasks.end(), _active_task.toStdString());

    return it - tasks.begin();
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
    return 0;
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

void JoyGuiBackEnd::setEnabledAxis(int idx, bool enabled) try
{
    ci();

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
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

qreal JoyGuiBackEnd::getMaxLinearSpeed() const
{
    return _max_linear;
}

qreal JoyGuiBackEnd::getMaxAngularSpeed() const
{
    return _max_angular;
}

void JoyGuiBackEnd::setMaxLinearSpeed(qreal vel) try
{
    ci();

    _max_linear = vel;

    cartesian_interface::SetJoystickTaskMaxSpeed srv;
    srv.request.max_linear_speed = vel;
    srv.request.max_angular_speed = getMaxAngularSpeed();

    if(!ros::service::call("cartesian/joystick/set_max_speed", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_max_speed'" << std::endl;
    }
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

void JoyGuiBackEnd::setMaxAngularSpeed(qreal vel) try
{
    ci();

    _max_angular = vel;

    cartesian_interface::SetJoystickTaskMaxSpeed srv;
    srv.request.max_linear_speed = getMaxLinearSpeed();
    srv.request.max_angular_speed = vel;

    if(!ros::service::call("cartesian/joystick/set_max_speed", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_max_speed'" << std::endl;
    }
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

QString JoyGuiBackEnd::getRefFrame() const try
{
    return _joy_map.at(_active_task.toStdString()).ref_frame;
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
    return "";
}

void JoyGuiBackEnd::setRefFrame(QString ref_frame) try
{
    ci();

    cartesian_interface::SetJoystickTaskBaseFrame srv;
    srv.request.base_frame = ref_frame.toStdString();
    if(!ros::service::call("cartesian/joystick/set_base_frame", srv))
    {
        std::cerr << "Unable to call service 'cartesian/joystick/set_base_frame'" << std::endl;
    }
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

void JoyGuiBackEnd::enableVelocityControl(bool enabled) try
{
    using CtrlMode = XBot::Cartesian::ControlType;
    ci()->setControlMode(_active_task.toStdString(), enabled ? CtrlMode::Velocity : CtrlMode::Position);
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

QString JoyGuiBackEnd::getBaseLink() const try
{
    return QString::fromStdString(ci()->getBaseLink(_active_task.toStdString()));
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
    return "MyTask1";
}

void JoyGuiBackEnd::setBaseLink(QString base_link) try
{
    ci()->setBaseLink(_active_task.toStdString(), base_link.toStdString());
}
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

JoyGuiBackEnd::ControlMode JoyGuiBackEnd::getControlMode(QString task) const try
{
    auto ci_ctrl = ci()->getControlMode(task.toStdString());

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
    return ControlMode::Position;
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

XBot::Cartesian::RosImpl::Ptr JoyGuiBackEnd::ci() const
{
    if(_ci)
    {
        return _ci;
    }

    throw std::runtime_error("CI ROS client unavailable");

}



void JoyGuiBackEnd::on_twist_recv(const geometry_msgs::TwistStampedConstPtr& msg, std::string task_id) try
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
catch(std::exception& e)
{
    std::cerr << "Error in " << __func__ << ": " << e.what() << std::endl;
}

void JoyGuiBackEnd::on_joy_status_recv(const cartesian_interface::JoystickStatusConstPtr & msg)
{
    if(msg->active_task != _active_task.toStdString())
    {
        setActiveTask(QString::fromStdString(msg->active_task));
    }

    _max_linear = msg->max_linear_speed;
    _max_angular = msg->max_angular_speed;

    emit joyStatusReceived();

}

JoyGuiBackEnd::JoyData::JoyData()
{

}
