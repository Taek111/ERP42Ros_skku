#ifndef __SUBSCRIBER_H__
#define __SUBSCRIBER_H__
#include <proxy/Connection.h>
#include <proxy/NodeHandle.h>
#include <vector>

namespace proxy
{

template <typename T>
class Subscriber : public HasSharedPtr<Subscriber<T>>
{
private:
    ros::Subscriber _sub;
    Connection::ptr_type _conn;
    std::vector<uint8_t> _serial_buf;
    Topic _topic;
    TopicMeta _t_meta;
    std::string _name;

public:
    Subscriber(Connection::ptr_type conn, const std::string &name)
        : _conn(conn), _name(name)
    {
        _sub = NodeHandle::get()->subscribe(name, 100, &Subscriber<T>::Callback, this);
        _topic._name = _name.c_str();
        _topic._meta = &_t_meta;
        _t_meta.hash = make_hash<T>();
        _t_meta.name_len = _name.size();

        ROS_INFO("Subscriber registered:");
        ROS_INFO("   name  :%s", _name.c_str());
        ROS_INFO("   hash1 :0x%lx", _t_meta.hash.value1);
        ROS_INFO("   hash2 :0x%lx", _t_meta.hash.value2);
    }
    void Callback(boost::shared_ptr<T const> msg)
    {
        _topic._meta->ser_len = ros::serialization::serializationLength(*msg);
        _serial_buf.resize(_topic._meta->ser_len);
        ros::serialization::OStream stream(_serial_buf.data(), _serial_buf.size());
        stream << *msg;
        _topic._ser_msg = _serial_buf.data();
        _conn->SendOneMsg(_topic);
        ROS_DEBUG("Send message");
    }
};
} // namespace proxy

#endif // __SUBSCRIBER_H__
