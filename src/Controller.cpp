#include "ros/ros.h"
#include <cmath>
#include "erp42_ros/target.h"
#include "erp42_ros/ERP42_input.h"
#include "erp42_ros/ERP42_feedback.h"

class Controller
{
public:
    typedef erp42_ros::target _target_type;
    typedef erp42_ros::ERP42_feedback _feedback_type;
    typedef erp42_ros::ERP42_input _ctrl_input_type;

    Controller(ros::NodeHandle &n)
        : ctrl_input_pub(n.advertise<_ctrl_input_type>("ctrl_input", 100)),
          target_sub(n.subscribe("target", 100, &Controller::OnTargetRecv, this)),
          feedback_sub(n.subscribe("feedback", 100, &Controller::OnFeedbackRecv, this)),
          loop_rate(50)

    { 
        // initialize target msg
        target_msg.brake = 50;
        target_msg.gear = 1;
        target_msg.speed_mps = 0;
        target_msg.steer_rad = 0;
    }
    void update()
    {
        input_msg.mode = 1;
        input_msg.Estop = false;
        input_msg.gear = target_msg.gear;
        input_msg.speed_mps = target_msg.speed_mps;
        input_msg.speed_kph = target_msg.speed_mps * 3600.0 / 1000.0;
        input_msg.steer_rad = target_msg.steer_rad;
        input_msg.steer_degree = target_msg.steer_rad * 180.0 / M_PI;
        input_msg.brake = target_msg.brake;
        

        ctrl_input_pub.publish(input_msg);
    }
    void spin()
    {
        while (ros::ok())
        {
            update();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void OnTargetRecv(const _target_type &target)
    {
        target_msg = target;
    }
    void OnFeedbackRecv(const _feedback_type &feedback)
    {
        feedback_msg = feedback;
    }

private:
    _target_type target_msg;
    _feedback_type feedback_msg;
    _ctrl_input_type input_msg;

    ros::Publisher ctrl_input_pub;
    ros::Subscriber target_sub;
    ros::Subscriber feedback_sub;
    ros::Rate loop_rate;
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle n;

    Controller ctrl(n);
    ctrl.spin();
    return 0;
}