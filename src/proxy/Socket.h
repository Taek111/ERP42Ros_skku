#ifndef Socket_h__
#define Socket_h__
#include <memory>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <unistd.h>
#include <netinet/tcp.h>
class EndPoint
{
private:
    sockaddr_in addr;
    socklen_t addr_len;

public:
    EndPoint()
    {
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr_len = sizeof(addr);
    }
    ~EndPoint()
    {
    }
    void set_host(const char *host)
    {
        addr.sin_addr.s_addr = inet_addr(host);
    }
    void set_port(unsigned short port)
    {
        addr.sin_port = htons(port);
    }
    socklen_t *get_len()
    {
        return &addr_len;
    }
    sockaddr_in *get_addr()
    {
        return &addr;
    }
};

class Socket
{
private:
protected:
    int fd;
    void usec2timeval(timeval *tv, unsigned int usec)
    {
        tv->tv_sec = usec / 1000000;
        tv->tv_usec = usec % 1000000;
    }

public:
    Socket() : fd(-1)
    {
    }

    ~Socket()
    {
        close(fd);
    }
    void Close()
    {
        close(fd);
    }
    bool is_readable_timeo(unsigned int usec)
    {
        timeval timo;
        usec2timeval(&timo, usec);
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd, &rset);
        int ret = select(fd + 1, &rset, nullptr, nullptr, &timo);
        if (ret > 0)
        {
            return true;
        }
        return false;
    }
    bool is_writable_timeo(unsigned int usec)
    {
        timeval timo;
        usec2timeval(&timo, usec);
        fd_set wset;
        FD_ZERO(&wset);
        FD_SET(fd, &wset);
        int ret = select(fd + 1, nullptr, &wset, nullptr, &timo);
        if (ret > 0)
        {
            return true;
        }
        return false;
    }
    bool is_valid()
    {
        int err;
        socklen_t len = sizeof(err);
        int ret = getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
        if ((ret < 0) || (err != 0))
        {
            return false;
        }
        return true;
    }

    bool Bind(EndPoint *ep)
    {
        return bind(fd, (const sockaddr *)ep->get_addr(), *(ep->get_len())) == 0;
    }
    bool Connect(EndPoint *ep)
    {
        return connect(fd,
                       (sockaddr *)ep->get_addr(),
                       *ep->get_len()) == 0;
    }
    ssize_t Send(const void *buf, size_t size)
    {
        return send(fd, buf, size, 0);
    }
    ssize_t SendTimeout(const void *buf, size_t size, unsigned int usec)
    {
        if (is_writable_timeo(usec))
        {
            return Send(buf, size);
        }
        return -1;
    }
    ssize_t Recv(void *buf, size_t size)
    {
        return recv(fd, buf, size, 0);
    }
    ssize_t Recvn(void *buf, size_t size)
    {
        return recv(fd, buf, size, MSG_WAITALL);
    }
    ssize_t RecvTimeout(void *buf, size_t size, unsigned int usec)
    {
        if (is_readable_timeo(usec))
        {
            return Recv(buf, size);
        }
        return -1;
    }
    ssize_t Peek(void *buf, size_t size)
    {
        return recv(fd, buf, size, MSG_DONTWAIT | MSG_PEEK);
    }
    ssize_t Peekn(void *buf, size_t size)
    {
        return recv(fd, buf, size, MSG_WAITALL | MSG_PEEK);
    }
    ssize_t PeekTimeout(void *buf, size_t size, unsigned int usec)
    {
        if (is_readable_timeo(usec))
        {
            return Peek(buf, size);
        }
        return -1;
    }
    ssize_t SendTo(const void *buf, size_t size, EndPoint *ep)
    {
        return sendto(fd, buf, size, 0,
                      (struct sockaddr *)ep->get_addr(),
                      *ep->get_len());
    }
    ssize_t SendToTimeout(void *buf, size_t size, EndPoint *ep, unsigned int usec)
    {
        if (is_writable_timeo(usec))
        {
            return SendTo(buf, size, ep);
        }
        return -1;
    }
    ssize_t RecvFrom(void *buf, size_t size, EndPoint *ep)
    {
        return recvfrom(fd, buf, size, 0, (struct sockaddr *)ep->get_addr(), ep->get_len());
    }
    ssize_t RecvFromTimeout(void *buf, size_t size, EndPoint *ep, unsigned int usec)
    {
        if (is_readable_timeo(usec))
        {
            return RecvFrom(buf, size, ep);
        }
        return -1;
    }
    ssize_t PeekFrom(void *buf, size_t size, EndPoint *ep)
    {
        return recvfrom(fd, buf, size, MSG_DONTWAIT | MSG_PEEK, (struct sockaddr *)ep->get_addr(), ep->get_len());
    }
    ssize_t PeekFromTimeout(void *buf, size_t size, EndPoint *ep, unsigned int usec)
    {
        if (is_readable_timeo(usec))
        {
            return PeekFrom(buf, size, ep);
        }
        return -1;
    }
};

class UDPSocket : public Socket
{
private:
    /* data */
public:
    UDPSocket(/* args */)
    {
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        int optval = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
    }

    ~UDPSocket()
    {
    }
};

class TCPSocket : public Socket
{
private:
    /* data */
public:
    TCPSocket(/* args */)
    {
        fd = socket(AF_INET, SOCK_STREAM, 0);
        int optval = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&optval, sizeof(int));
    }
    TCPSocket(int new_fd)
    {
        fd = new_fd;
    }
    ~TCPSocket()
    {
    }
    bool Listen()
    {
        return listen(fd, 5) == 0;
    }
    TCPSocket *Accept(EndPoint *remote_ep)
    {
        int new_fd = accept(fd,
                            (sockaddr *)remote_ep->get_addr(),
                            remote_ep->get_len());
        if (new_fd < 0)
        {
            return nullptr;
        }
        return new TCPSocket(new_fd);
    }
    TCPSocket *AcceptTimeout(EndPoint *remote_ep, unsigned int usec)
    {
        bool readable = is_readable_timeo(usec);
        if (readable)
        {
            return Accept(remote_ep);
        }
        return nullptr;
    }
};

// class Connection
// {
//   private:
//     Socket sock;

//   public:
// };

#endif //Socket_h__