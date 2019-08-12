#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/scoped_ptr.hpp>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define LINEAR 0
#define ANGULAR 1

#define MAX_LINEAR_VELOCITY 5.55 // (m / s), approximation of 20 km/h
#define MIN_LINEAR_VELOCITY -MAX_LINEAR_VELOCITY

using namespace std;

class erp42Core
{
public:
    erp42Core(ros::NodeHandle &nh)
        :cmd_vel_sub(nh.subscribe("cmd_vel",5, &erp42Core:: commandVelocityCallback, this)),
        odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 5)),
        loop_rate(10)
    {
        initOdom();
    }

    void update()
    {
        publishDriveInformation();
    }

    void spin()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            update();
            loop_rate.sleep();
        }
    }

    void initOdom(void)
    {
        for(int i = 0; i < 3; i++)
        {
            odom_pose[i] = 0.0;
            odom_vel[i] = 0.0;
        }
        
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 0.0;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.angular.z = 0.0;
    }

    void publishDriveInformation(void)
    {
        ros::Time time_now = ros::Time::now();
        ros::Duration step = (time_now - prev_update_time);
        double step_time = (double)(step.sec) + (double)(step.nsec)*1e-9;
        prev_update_time = time_now;
        
        calcOdometry(step_time);

        updateOdometry();
        odom.header.stamp = time_now;
        odom_pub.publish(odom);
    
        //TODO: odom tf 관련 정보 publish 
        updateTF(odom_tf);
        odom_tf.header.stamp = time_now;
        tf_broadcaster_.sendTransform(odom_tf);
    }

    //TODO: 일단은 터틀봇과 같은 2 Wheel differential drive robot으로 생각했으나 
    // 엔코더를 erp42에 맞게  적용한 후 odometry 계산법을 바꿔야함 
    void calcOdometry(double diff_time)
    {
        double step_time = diff_time;
        double delta_s, delta_theta;
        delta_s = goal_velocity_from_cmd[LINEAR] * step_time;
        delta_theta = goal_velocity_from_cmd[ANGULAR] * step_time;

        odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
        odom_pose[1] += delta_s * sin(odom_pose[2] * (delta_theta / 2.0));
        odom_pose[2] += delta_theta;

        odom_vel[0] =  goal_velocity_from_cmd[LINEAR];
        odom_vel[1] = 0.0;
        odom_vel[2] =  goal_velocity_from_cmd[ANGULAR];
    }

    void updateOdometry(void)
    {
        odom.header.frame_id = odom_header_frame_id;
        odom.child_frame_id = odom_child_frame_id;

        odom.pose.pose.position.x = odom_pose[0];
        odom.pose.pose.position.y = odom_pose[1];
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

        odom.twist.twist.linear.x = odom_vel[0];
        odom.twist.twist.angular.z = odom_vel[2];
        
    }

    void updateTF(geometry_msgs::TransformStamped& odom_tf)
    {
        odom_tf.header = odom.header;
        odom_tf.child_frame_id = odom.child_frame_id;
        odom_tf.transform.translation.x = odom.pose.pose.position.x;
        odom_tf.transform.translation.y = odom.pose.pose.position.y;
        odom_tf.transform.translation.z = odom.pose.pose.position.z;
        odom_tf.transform.rotation = odom.pose.pose.orientation;
    }

    void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
    {
        if (cmd_vel_msg.linear.x > MAX_LINEAR_VELOCITY)
            goal_velocity_from_cmd[LINEAR] = MAX_LINEAR_VELOCITY;
        else if (cmd_vel_msg.linear.x < MIN_LINEAR_VELOCITY)
            goal_velocity_from_cmd[LINEAR] = MIN_LINEAR_VELOCITY;
        else
            goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
        ROS_INFO("MAX:%f, min:%f", MAX_LINEAR_VELOCITY, MIN_LINEAR_VELOCITY);
        ROS_INFO("%f, %f", goal_velocity_from_cmd[LINEAR], cmd_vel_msg.linear.x);
        goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
    }

private:
    /*Subscriber */
    ros::Subscriber cmd_vel_sub;
    /*Publisher */
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub;
    /*Transform Broadcaster */
    geometry_msgs::TransformStamped odom_tf;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /*Parameter*/
    float goal_velocity_from_cmd[2] = {0.0, 0.0};
    float odom_pose[3];
    double odom_vel[3];

    char odom_header_frame_id[30] = "odom";
    char odom_child_frame_id[30] = "base_footprint";

    ros::Time prev_update_time;
    ros::Rate loop_rate;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erp42_core");
    ros::NodeHandle nh;
    erp42Core core(nh);
    core.spin();
    return 0;
}