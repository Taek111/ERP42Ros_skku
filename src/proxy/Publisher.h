#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__
#include <proxy/traits.h>
#include <proxy/NodeHandle.h>
#include <proxy/Topic.h>

namespace proxy
{

// this class is interface of type specialized publishers.
// PublisherBase Publishes Topic from Serialized messages

class PublisherBase
    : public HasSharedPtr<PublisherBase>
{
protected:
    ros::Publisher _pub;

public:
    PublisherBase() {}
    virtual ~PublisherBase() = default;
    virtual void publish(Topic &topic) = 0;
};

// this is specialized Publishers.
template <typename T>
class Publisher
    : public PublisherBase,
      public HasSharedPtr<Publisher<T>>
{

public:
    Publisher(std::string name, uint32_t queue_size = 100)
    {
        std::string full_name = "proxy/";
        _pub = NodeHandle::get()->advertise<T>(full_name + name, queue_size);
    }

    virtual void publish(Topic &topic)
    {
        ros::serialization::IStream istream(topic._ser_msg, topic._meta->ser_len);
        istream >> _msg;
        _pub.publish(_msg);
    }

private:
    T _msg;
};

};     //namespace proxy
#endif // __PUBLISHER_H__
