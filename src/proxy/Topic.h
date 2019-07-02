#ifndef __TOPIC_H__
#define __TOPIC_H__
#include <memory.h>
#include <ros/ros.h>
#include <proxy/traits.h>

namespace proxy
{
struct Hash
{
    uint64_t value1;
    uint64_t value2;
    bool operator==(const Hash &other) const
    {
        return value1 == other.value1 && value2 == other.value2;
    }
    bool operator<(const Hash &other) const
    {
        if (value1 < other.value1)
            return true;
        else if (value1 == other.value1 && value2 < other.value2)
            return true;
        return false;
    }
    // bool operator==(Hash::ptr_type other) const
    // {
    //     return operator==(*other);
    // }
    // bool operator<(Hash::ptr_type other) const
    // {
    //     return operator<(*other);
    // }
};

template <typename T>
inline Hash make_hash()
{
    Hash ret;
    ret.value1 = ros::message_traits::MD5Sum<T>::static_value1;
    ret.value2 = ros::message_traits::MD5Sum<T>::static_value2;
    return ret;
};

struct TopicMeta
{
    uint16_t name_len;
    uint16_t ser_len;
    Hash hash;
};

struct Topic
{
    TopicMeta *_meta;
    const char *_name;
    uint8_t *_ser_msg;
    Topic()
        : _meta(nullptr), _name(nullptr), _ser_msg(nullptr)
    {
    }
    Topic(TopicMeta *meta, const char *name, uint8_t *ser_msg)
        : _meta(meta), _name(name), _ser_msg(ser_msg)
    {
    }
    Topic(void *serialized)
    {
        _meta = (TopicMeta *)serialized;
        _name = (char *)serialized + sizeof(TopicMeta);
        _ser_msg = (uint8_t *)serialized + sizeof(TopicMeta) + _meta->name_len;
    }
    int Serialize(void *buf, size_t buf_size)
    {
        if (buf_size < SerialLen())
            return -1;
        memcpy((TopicMeta *)buf, _meta, sizeof(*_meta));
        memcpy((char *)buf + sizeof(TopicMeta), _name, _meta->name_len);
        memcpy((char *)buf + sizeof(TopicMeta) + _meta->name_len, _ser_msg, _meta->ser_len);
        return SerialLen();
    }
    int SerialLen()
    {
        return sizeof(TopicMeta) + _meta->name_len + _meta->ser_len;
    }
};

// this serialzier will serialize your ros msg into send buffer directly.
// this class makes you avoid unnecessary copying.

};     // namespace proxy
#endif // __TOPIC_H__