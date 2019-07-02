#ifndef __NODEHANDLE_H__
#define __NODEHANDLE_H__
#include <ros/ros.h>

namespace proxy
{
struct NodeHandle
{
    static std::shared_ptr<ros::NodeHandle> get()
    {
        static std::shared_ptr<ros::NodeHandle> node(nullptr);
        if (!node)
            node = std::make_shared<ros::NodeHandle>();
        return node;
    }
};
} // namespace proxy

#endif // __NODEHANDLE_H__
