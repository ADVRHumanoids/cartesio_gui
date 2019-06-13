#ifndef _QTUTILS_H_
#define _QTUTILS_H_

#include <ros/ros.h>

namespace cartesio_gui
{
    namespace qt_utils
    {
    template <class data_type>
    class RosInterface
    {
    public:
        RosInterface(const std::string& interested_data_type):
            _data_type(interested_data_type)
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
                                     (_topics[i], _n.advertise<data_type>(_topics[i], 1)) );
            }
        }

        std::vector<std::string> getTopics(){return _topics;}

        void publish(const std::string& topic, const data_type& msg)
        {
            _publishers[topic].publish(msg);
        }
    private:
        std::string _data_type;
        std::vector<std::string> _topics;
        ros::NodeHandle _n;
        std::map<std::string, ros::Publisher> _publishers;

    };

    }
}

#endif
