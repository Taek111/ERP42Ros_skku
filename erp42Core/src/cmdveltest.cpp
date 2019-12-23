#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmdveltest");
    ros::NodeHandle n;

    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(20);

    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;//0.5
    ros::Time beginTime = ros::Time::now();
    ros::Duration duration = ros::Duration(15);
    ros::Time endTime = beginTime + duration;

    while (ros::Time::now() < endTime)
    {
        velPub.publish(msg);
        loop_rate.sleep();
    }
    msg.linear.x = 0.0;
    ros::Time beginTime1 = ros::Time::now();
    while (ros::Time::now() < beginTime1+ros::Duration(1))
    {
        velPub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
