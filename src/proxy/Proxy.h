#ifndef __PROXY_H__
#define __PROXY_H__

#include <ros/ros.h>
#include <proxy/Factory.h>
#include <proxy/Subscriber.h>
#include <proxy/Connection.h>

class Proxy
{
private:
    EndPoint _local, _remote;
    proxy::FactoryMap::ptr_type _fmap;
    proxy::TopicMap::ptr_type _tmap;
    proxy::Connection::ptr_type _conn;
    proxy::Topic _topic;

public:
    Proxy(uint16_t local_port, const std::string &remote_host, uint16_t remote_port)
    {
        _local.set_port(local_port);
        _remote.set_port(remote_port);
        _remote.set_host(remote_host.c_str());
        _fmap = std::make_shared<proxy::FactoryMap>();
        _tmap = std::make_shared<proxy::TopicMap>(_fmap);
        _conn = std::make_shared<proxy::Connection>(&_local, &_remote);
    }
    template <typename T>
    void addPubFactory()
    {
        _fmap->addPubFactory<T>();
    }

    template <typename T>
    std::shared_ptr<proxy::Subscriber<T>> getSubscriber(const std::string &topic_name)
    {
        return std::make_shared<proxy::Subscriber<T>>(_conn, topic_name);
    }

    void spin()
    {
        while (ros::ok())
        {
            int ret = _conn->RecvOneMsg(_topic);
            if (ret > 0)
            {
                _tmap->Publish(_topic);
            }
            ros::spinOnce();
        }
    }
};

class ROSParamHelper
{
private:
    std::string _node_name;

public:
    ROSParamHelper()
        : _node_name(ros::this_node::getName())
    {
    }

    template <typename T>
    T getParam(const std::string &param_name)
    {
        T dst;
        std::string full_name = _node_name + "/" + param_name;
        assert_param(full_name);
        proxy::NodeHandle::get()->getParam(full_name, dst);
        return dst;
    }

private:
    void assert_param(const std::string &full_name)
    {
        if (!proxy::NodeHandle::get()->hasParam(full_name))
        {
            ROS_INFO("%s is not defined", full_name.c_str());
            throw;
        }
    }
};

#endif // __PROXY_H__
