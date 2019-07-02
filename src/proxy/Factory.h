#ifndef __FACTORY_H__
#define __FACTORY_H__
#include <proxy/traits.h>
#include <map>
#include <proxy/Topic.h>
#include <proxy/Publisher.h>

namespace proxy
{

class PubFactoryBase : public HasSharedPtr<PubFactoryBase>
{
protected:
public:
    PubFactoryBase() = default;
    virtual ~PubFactoryBase() = default;
    virtual PublisherBase::ptr_type getPublisher(const std::string &name) = 0;
};

template <typename T>
class PubFactory : public PubFactoryBase, public HasSharedPtr<PubFactory<T>>
{
public:
    PubFactory() = default;
    virtual ~PubFactory() = default;
    virtual PublisherBase::ptr_type getPublisher(const std::string &name)
    {
        auto p = std::make_shared<Publisher<T>>(name, 100);
        return std::static_pointer_cast<PublisherBase>(p);
    }
};

class FactoryMap : public HasSharedPtr<FactoryMap>
{
private:
    std::map<Hash, PubFactoryBase::ptr_type> _map;

public:
    FactoryMap() = default;
    ~FactoryMap() = default;
    PubFactoryBase::ptr_type getPubFactory(const Hash &hash)
    {
        if (_map.count(hash) > 0)
        {
            return _map[hash];
        }
        return PubFactoryBase::ptr_type(nullptr);
    }
    template <typename T>
    void addPubFactory()
    {
        auto hash = make_hash<T>();
        if (_map.count(hash) > 0)
            return;
        auto p = std::make_shared<PubFactory<T>>();
        _map[hash] = std::static_pointer_cast<PubFactoryBase>(p);
    }
};

class TopicMap : public HasSharedPtr<TopicMap>
{
private:
    std::map<std::string, PublisherBase::ptr_type> _map;
    FactoryMap::ptr_type _fmap;

public:
    TopicMap(FactoryMap::ptr_type &fmap) : _fmap(fmap) {}
    virtual ~TopicMap() {}
    void Publish(Topic &topic)
    {
        std::string topic_name(topic._name, topic._meta->name_len);
        if (_map.count(topic_name) == 0)
        {
            auto pubfac = _fmap->getPubFactory(topic._meta->hash);
            // ROS_INFO("hash1 : %lx", topic._hash.value1);
            // ROS_INFO("hash2 : %lx", topic._hash.value2);
            if (pubfac.get() == nullptr)
            {
                ROS_INFO("this type of message has not been registered.");
                return;
            }
            ROS_INFO("Detect New topic. Generate publisher from factory.");
            _map[topic_name] = pubfac->getPublisher(topic_name);
        }
        _map[topic_name]->publish(topic);
    }
};

} // namespace proxy

#endif // __FACTORY_H__
