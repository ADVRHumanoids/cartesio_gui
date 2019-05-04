#include <ros/ros.h>
#include <cartesian_interface/ros/RosImpl.h>

#include <QWidget>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QLineEdit>
#include <QFrame>
#include <QGroupBox>
#include <QRadioButton>

namespace cartesio_gui
{
    class TaskOptionsWidget : public QMainWindow 
    {
        
    public:
        
        TaskOptionsWidget(QWidget * parent,  
                          ros::NodeHandle nh
                        ); 
        
    private:
        
        void construct();
        void update();
        void apply_lims();
        
        void on_task_changed(const QString& task);
        void on_base_link_changed();
        void on_ctrl_mode_changed(XBot::Cartesian::ControlType ctrl_mode);
        void on_safety_lims_changed();
        
        std::string get_current_task() const;
        std::string get_control_mode() const;
        std::string get_base_link() const;
        
        ros::NodeHandle _nh;

        XBot::Cartesian::RosImpl::Ptr _ci;
        
        QFrame * _central_frame;
        
        QMetaObject::Connection _w_task_list_connection;
        QComboBox * _w_task_list;
        QLineEdit * _w_base_link_field;
        
        std::pair<QLineEdit *, QLineEdit *> _w_vel_lims;
        std::pair<QLineEdit *, QLineEdit *> _w_acc_lims;
        
        QRadioButton * _w_pos_mode;
        QRadioButton * _w_vel_mode;
        QRadioButton * _w_dis_mode;
        
        QPushButton * _w_button_apply_lims;
       
        QPushButton * _w_button_update_all;
    };
    


}
