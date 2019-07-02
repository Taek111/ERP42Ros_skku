#ifndef __CONNECTION_H__
#define __CONNECTION_H__
#include <proxy/Socket.h>
#include <proxy/traits.h>
#include <proxy/Topic.h>

namespace proxy
{
class Connection : public HasSharedPtr<Connection>
{
private:
    struct Header
    {
        uint16_t total_size;
        uint16_t sequence;
    };

private:
    UDPSocket _sock;
    EndPoint *_remote;
    uint16_t sequence;
    char _recv_buf[2048];
    char _send_buf[2048];

public:
    Connection(EndPoint *local, EndPoint *remote)
        : _remote(remote)
    {
        _sock.Bind(local);
        Header *header = (Header *)_send_buf;
        header->sequence = -1;
    }
    virtual ~Connection()
    {
    }
    virtual int RecvOneMsg(Topic &topic)
    {
        ssize_t sz = _sock.RecvFromTimeout(
            _recv_buf, sizeof(_recv_buf), _remote, 100);

        // reject read failure
        if (sz < 0)
        {
            return -1;
        }

        // reject incomplete packet
        Header *header = (Header *)_recv_buf;
        if (sz != header->total_size)
        {
            ROS_DEBUG("Receive size not match");
            return -1;
        }

        // reject multiple receive
        if (sequence == header->sequence)
        {
            ROS_DEBUG("Receive multiple packet");
            return -1;
        }

        topic = Topic(_recv_buf + sizeof(Header));
        return sz;
    };
    virtual int SendOneMsg(Topic &topic)
    {
        // make header
        Header *header = (Header *)_send_buf;
        header->sequence = header->sequence + 1;
        header->total_size = sizeof(Header) + topic.SerialLen();
        // make payload
        topic.Serialize(_send_buf + sizeof(Header), 2048 - sizeof(Header));
        //send
        return _sock.SendTo(_send_buf, header->total_size, _remote);
    };
};

} // namespace proxy

#endif // __CONNECTION_H__
