#include <cartesio_gui/cartesio/task_options_widget.h>

#include <QToolBar>
#include <QIcon>
#include <QAction>
#include <QStatusBar>

using namespace XBot::Cartesian;

cartesio_gui::TaskOptionsWidget::TaskOptionsWidget(QWidget * parent,
                                                   ros::NodeHandle nh):
    QMainWindow(parent),
    _nh(nh)
{

    // central frame of main window
    _central_frame = new QFrame;
    
    // first frame contains the menu for task selection
    auto * frame_task_list = new QGroupBox("Task selection");
    _w_task_list = new QComboBox;
    _w_task_list->setMinimumWidth(130);
    
    auto * frame_task_list_layout = new QHBoxLayout;
    frame_task_list_layout->addWidget(new QLabel("Selected task:"));
    frame_task_list_layout->addWidget(_w_task_list);
    frame_task_list->setLayout(frame_task_list_layout);
    
    /* Create two more frames, one for base link / control mode selection,
     * and the second for safety limits adjustments */

    // base link selection frame
    auto * base_link_label = new QLabel("Task base link:");
    _w_base_link_field = new QLineEdit;
    
    connect(_w_base_link_field, &QLineEdit::editingFinished, 
            this, &TaskOptionsWidget::on_base_link_changed);
    
    auto * frame_task_properties_layout = new QVBoxLayout;
    auto * frame_base_link = new QGroupBox("Base link selection");
    auto * frame_base_link_layout = new QHBoxLayout;
    frame_base_link_layout->addWidget(base_link_label);
    frame_base_link_layout->addWidget(_w_base_link_field);
    frame_base_link->setLayout(frame_base_link_layout);
    
    // control mode selection frame
    _w_pos_mode = new QRadioButton("Position");
    _w_vel_mode = new QRadioButton("Velocity");
    _w_dis_mode = new QRadioButton("Disabled");
    
    connect(_w_pos_mode, &QRadioButton::pressed, 
            std::bind(&TaskOptionsWidget::on_ctrl_mode_changed, this, 
                      CartesianInterface::ControlTypeFromString("Position")));
    
    connect(_w_vel_mode, &QRadioButton::pressed, 
            std::bind(&TaskOptionsWidget::on_ctrl_mode_changed, this, 
                      CartesianInterface::ControlTypeFromString("Velocity")));
    
    connect(_w_dis_mode, &QRadioButton::pressed, 
            std::bind(&TaskOptionsWidget::on_ctrl_mode_changed, this,
                      CartesianInterface::ControlTypeFromString("Disabled")));
    
    
    auto * radio_buttons_layout = new QVBoxLayout;
    radio_buttons_layout->addWidget(_w_pos_mode);
    radio_buttons_layout->addWidget(_w_vel_mode);
    radio_buttons_layout->addWidget(_w_dis_mode);
    radio_buttons_layout->addStretch(1);
    
    auto * button_group = new QGroupBox("Control mode selection");
    button_group->setLayout(radio_buttons_layout);
    
    frame_task_properties_layout->addLayout(frame_base_link_layout);
    frame_task_properties_layout->addWidget(button_group);

    // safety limits tuning frame
    _w_vel_lims.first = new QLineEdit("0");
    _w_vel_lims.second = new QLineEdit("0");
    _w_acc_lims.first = new QLineEdit("0");
    _w_acc_lims.second = new QLineEdit("0");
    
    connect(_w_vel_lims.first, &QLineEdit::editingFinished, 
            this, &TaskOptionsWidget::on_safety_lims_changed);
    connect(_w_vel_lims.second, &QLineEdit::editingFinished, 
            this, &TaskOptionsWidget::on_safety_lims_changed);
    connect(_w_acc_lims.first, &QLineEdit::editingFinished, 
            this, &TaskOptionsWidget::on_safety_lims_changed);
    connect(_w_acc_lims.second, &QLineEdit::editingFinished, 
            this, &TaskOptionsWidget::on_safety_lims_changed);
    
    
    auto * frame_lims_layout = new QGridLayout;
    
    frame_lims_layout->setRowStretch(2, 1);
    
    frame_lims_layout->addWidget(new QLabel("Velocity (lin./ang.)"), 0, 0);
    frame_lims_layout->addWidget(_w_vel_lims.first, 0, 1);
    frame_lims_layout->addWidget(_w_vel_lims.second, 0, 2);
    
    frame_lims_layout->addWidget(new QLabel("Accel.   (lin./ang.)"), 1, 0);
    frame_lims_layout->addWidget(_w_acc_lims.first, 1, 1);
    frame_lims_layout->addWidget(_w_acc_lims.second, 1, 2);
    
    
    auto * frame_lims = new QGroupBox("Safety limits");
    frame_lims->setLayout(frame_lims_layout);
    
    /* Main layout */
    auto * main_layout = new QGridLayout;
    main_layout->addWidget(frame_task_list, 0, 0);
    main_layout->addLayout(frame_task_properties_layout, 1, 0);
    main_layout->addWidget(frame_lims, 1, 1);
    
    _central_frame->setLayout(main_layout);
    
    setCentralWidget(_central_frame);
    statusBar()->show();
    
    auto * toolbar = addToolBar("Main toolbar");
    auto icon = style()->standardIcon(QStyle::SP_DialogResetButton);
    auto * reload_action = toolbar->addAction(icon, "Reload");
    
    connect(reload_action, &QAction::triggered, this, &TaskOptionsWidget::construct);
    
    construct();
}


void cartesio_gui::TaskOptionsWidget::construct()
{
    statusBar()->showMessage("Loading CI object..");
    
    try
    {
        // obtain ci
        _ci = std::make_shared<XBot::Cartesian::RosImpl>();

        auto task_list = _ci->getTaskList();
        
        disconnect(_w_task_list_connection);
        
        _w_task_list->clear();
        
        _w_task_list_connection = connect(_w_task_list, &QComboBox::currentTextChanged, 
                                          this, &TaskOptionsWidget::on_task_changed);
        
        for(auto t : task_list)
        {
            _w_task_list->addItem(QString::fromStdString(t));
        }
        
        update();
        
        _central_frame->setEnabled(true);
        
        statusBar()->showMessage("Ready");
        
        
    }
    catch(std::exception& e)
    {
        statusBar()->showMessage("Loading CI object failed: " + QString::fromUtf8(e.what()));
        _central_frame->setEnabled(false);
    }
    
    
    
    
}

void cartesio_gui::TaskOptionsWidget::update()
{
    // update base link
    _w_base_link_field->setText(QString::fromStdString(_ci->getBaseLink(get_current_task())));
    
    // update control mode
    auto ctrl = _ci->getControlMode(get_current_task());
    
    switch(ctrl)
    {
        case XBot::Cartesian::ControlType::Position:
        {
            _w_pos_mode->setChecked(true);
            break;
        }
        
        case XBot::Cartesian::ControlType::Velocity:
        {
            _w_vel_mode->setChecked(true);
            break;
        }
        
        case XBot::Cartesian::ControlType::Disabled:
        {
            _w_dis_mode->setChecked(true);
            break;
        }
        
        default:
        {
            throw std::runtime_error("Invalid control mode");
        }
        
    }
    
    // update safety limits
    double lin = -1., ang = -1.;
    int field_width = 4;
    int digits = 1;
    
    _ci->getVelocityLimits(get_current_task(), lin, ang);
    _w_vel_lims.first->setText(QString("%1").arg(lin, field_width, 'f', digits));
    _w_vel_lims.second->setText(QString("%1").arg(ang, field_width, 'f', digits));
    
    _ci->getAccelerationLimits(get_current_task(), lin, ang);
    _w_acc_lims.first->setText(QString("%1").arg(lin, field_width, 'f', digits));
    _w_acc_lims.second->setText(QString("%1").arg(ang, field_width, 'f', digits));
}

std::string cartesio_gui::TaskOptionsWidget::get_current_task() const
{
    return _w_task_list->currentText().toStdString();
}


void cartesio_gui::TaskOptionsWidget::apply_lims()
{
}

std::string cartesio_gui::TaskOptionsWidget::get_base_link() const
{
}

std::string cartesio_gui::TaskOptionsWidget::get_control_mode() const
{
}

void cartesio_gui::TaskOptionsWidget::on_base_link_changed()
{
    try
    {
        _ci->setBaseLink(get_current_task(),
                         _w_base_link_field->text().toStdString());
        
        statusBar()->showMessage("Successfully changed base link");
    }
    catch(std::exception& e)
    {
        statusBar()->showMessage(e.what());
    }
}

void cartesio_gui::TaskOptionsWidget::on_ctrl_mode_changed(XBot::Cartesian::ControlType ctrl_mode)
{
    try
    {
        _ci->setControlMode(get_current_task(),
                            ctrl_mode);
        
        statusBar()->showMessage("Successfully changed control mode");
    }
    catch(std::exception& e)
    {
        statusBar()->showMessage(e.what());
    }
}

void cartesio_gui::TaskOptionsWidget::on_safety_lims_changed()
{
    
    bool vl, va, al, aa;
    double vl_value = _w_vel_lims.first->text().toDouble(&vl);
    double va_value = _w_vel_lims.second->text().toDouble(&va);
    double al_value = _w_acc_lims.first->text().toDouble(&al); 
    double aa_value = _w_acc_lims.second->text().toDouble(&aa);
    
    if(!vl)
    {
        statusBar()->showMessage("Formatting error: linear velocity limit is not a valid float");
        return;
    }
    
    if(!va)
    {
        statusBar()->showMessage("Formatting error: angular velocity limit is not a valid float");
        return;
    }
    
    if(!al)
    {
        statusBar()->showMessage("Formatting error: linear acceleration limit is not a valid float");
        return;
    }
    
    if(!aa)
    {
        statusBar()->showMessage("Formatting error: angular acceleration limit is not a valid float");
        return;
    }
    
    try
    {
        _ci->setVelocityLimits(get_current_task(), vl_value, va_value);
        _ci->setAccelerationLimits(get_current_task(), al_value, aa_value);
        statusBar()->showMessage("Successfully updated safety limits");
    }
    catch(std::exception& e)
    {
        statusBar()->showMessage(e.what());
    }
    
}

void cartesio_gui::TaskOptionsWidget::on_task_changed(const QString& task)
{
    update();
}




