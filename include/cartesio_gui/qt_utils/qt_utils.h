#ifndef _QTUTILS_H_
#define _QTUTILS_H_

#include <ros/ros.h>
#include <QtUiTools>
#include <QWidget>
#include <functional>
#include <iostream>

namespace cartesio_gui
{
    namespace qt_utils
    {

        template <class msg_type>
        class RosSpecialPublisher
        {
        public:
            RosSpecialPublisher(const std::string& msg_data_type):
                _data_type(msg_data_type)
            {
                XmlRpc::XmlRpcValue args, result, payload;
                args[0] = ros::this_node::getName();
                if (!ros::master::execute("getTopicTypes", args, result, payload, true))
                {
                    throw std::runtime_error("Can not execute getSystemState");
                }
                for(unsigned int j = 0; j < payload.size(); ++j)
                {
                    if(payload[j][1] == _data_type)
                        _topics.push_back(payload[j][0]);
                }


                std::cout<<"Available topics:"<<std::endl;
                for(unsigned int i = 0; i < _topics.size(); ++i)
                {
                    std::cout<<_topics[i]<<std::endl;
                    _publishers.insert ( std::pair<std::string,ros::Publisher>
                                         (_topics[i], _n.advertise<msg_type>(_topics[i], 1)) );
                }
            }

            std::vector<std::string> getTopics(){return _topics;}

            bool publish(const std::string& topic, const msg_type& msg)
            {
                if(!_publishers[topic])
                    return false;

                _publishers[topic].publish(msg);
                return true;
            }
        private:
            std::string _data_type;
            std::vector<std::string> _topics;
            ros::NodeHandle _n;
            std::map<std::string, ros::Publisher> _publishers;

        };



        template <class msg_type>
        /**
         * @brief The RosPublisherWidget class implements a widget with a topic slector and a button which
         * enable/disable a periodic publisher
         */
        class RosPublisherWidget: public QWidget,
                                  public RosSpecialPublisher<msg_type>
        {
        public:
            typedef std::shared_ptr< RosPublisherWidget<msg_type> > Ptr;

            /**
             * @brief RosPublisherWidget
             * @param dT_ms period of publisher (in ms)
             * @param msg_data_type ex: "geometry_msgs/WrenchStamped"
             * @param parent_widget
             */
            RosPublisherWidget(const int dT_ms, const std::string& msg_data_type, QWidget* parent_widget = 0):
                QWidget(parent_widget),
                RosSpecialPublisher<msg_type>(msg_data_type),
                _dT_ms(dT_ms)
            {
                /* Create GUI layout */
                auto * ui = LoadUiFile(this);

                auto * layout = new QVBoxLayout;
                layout->addWidget(ui);

                setLayout(layout);

                _send_button = findChild<QPushButton *>("pushButton");
                _send_button->setText("Send");
                _send_reference = false;
                if(this->getTopics().size() == 0)
                    _send_button->setEnabled(false);

                connect(_send_button, &QPushButton::clicked, this, &RosPublisherWidget::on_send_button_clicked);

                _topic_selector = findChild<QComboBox *>("comboBox");
                fillTopicSelector(this->getTopics());

                void (QComboBox::* currentIndexChangedInt)(int) = &QComboBox::currentIndexChanged;

                connect(_topic_selector, currentIndexChangedInt, this, &RosPublisherWidget::on_topic_changed);
                if(this->getTopics().size() > 0)
                {
                    _topic_selector->setCurrentIndex(0);
                    on_topic_changed(0);
                }


                _timer = new QTimer(this);
                connect(_timer, &QTimer::timeout, this, &RosPublisherWidget::update);
                _timer->start(_dT_ms); //ms
            }

            /**
             * @brief fillMsg
             * @param msg which will be published
             */
            void fillMsg(const msg_type& msg)
            {
                _msg = msg;
            }

        private:

            QWidget * LoadUiFile(QWidget * parent)
            {
                QUiLoader loader;

                QFile file(":/ui/ros_publisher_widget.ui");
                file.open(QFile::ReadOnly);

                QWidget *formWidget = loader.load(&file, parent);
                file.close();

                return formWidget;
            }

            QPushButton* _send_button;
            bool _send_reference;

            int _dT_ms;

            void on_send_button_clicked(bool checked)
            {
                if(_send_reference)
                {
                    _send_reference = false;
                    _send_button->setText("Send");
                    std::cout<<"STOP sending references"<<std::endl;
                }
                else
                {
                    _send_reference = true;
                    _send_button->setText("Stop");
                    std::cout<<"START sending references"<<std::endl;
                }
            }

            QComboBox* _topic_selector;
            QStringList _available_topics;

            void fillTopicSelector(const std::vector<std::string>& topics)
            {
                if(topics.size() > 0)
                {
                    for(unsigned int i = 0; i < topics.size(); ++i)
                    {
                        QString s(topics[i].c_str());
                        _available_topics.push_back(s);
                    }

                    for(unsigned int i = 0; i < _available_topics.size(); ++i)
                        _topic_selector->insertItem(i, _available_topics[i]);
                }
                else {
                    _topic_selector->insertItem(0, "none");
                }
            }


            void on_topic_changed(int i)
            {
                _topic = _topic_selector->itemText(i);
                std::cout<<"Publish to: "<<_topic.toStdString()<<std::endl;
            }

            QString _topic;
            msg_type _msg;

            QTimer* _timer;

            void update()
            {
                if(_send_reference && this->getTopics().size() > 0)
                {
                    this->publish(_topic.toStdString(), _msg);
                }

                ros::spinOnce();
            }
        };



    /**
     * @brief The BoxedSliderWidget class provides a widget with integrated a slider and a double spin box which
     * are updated each other.
     */
    class BoxedSliderWidget: public QWidget
    {
    public:
        typedef std::shared_ptr<BoxedSliderWidget> Ptr;

        BoxedSliderWidget(QWidget* parent_widget = 0);

        /**
         * @brief getValue
         * @return actual value of the slider and double spin box
         */
        double getValue(){return _value;}

        /**
         * @brief on_value_changed_call_back implements here your code to be called every time the value in the
         * slider or double spin box change
         * @param value of the slider and double spin box
         */
        virtual void on_value_changed_call_back(const int value)
        {}

        /**
         * @brief setLimits of the slider and double spin box
         * @param min
         * @param max
         */
        void setLimits(int min, int max);

        /**
         * @brief setLable
         * @param label of the slider
         */
        void setLable(const QString& label);

        /**
         * @brief setValue of both slider and double spin box
         * @param value
         */
        void setValue(int value);

    private:
        QWidget * LoadUiFile(QWidget * parent)
        {
            QUiLoader loader;

            QFile file(":/ui/boxed_slider.ui");
            file.open(QFile::ReadOnly);

            QWidget *formWidget = loader.load(&file, parent);
            file.close();

            return formWidget;
        }

        QSlider* _slider;
        QDoubleSpinBox* _spin_box;
        QWidget* _parent_widget;
        QLabel* _label;

        double _value;

        int _min, _max;

        void on_slider_value_changed(int value);

        void on_box_value_changed();

    };
}
}

#endif
