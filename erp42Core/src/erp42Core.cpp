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
#include "erp42_slam2d/ERP42_feedback.h"
#include "erp42_slam2d/ERP42_input.h"

#define TICK2RAD 0.06283185307 // 3.6[deg] * 3.14159265359 / 180 = 0.06283185307
#define WHEEL_RADIUS 0.265           // meter
#define WHEEL_SEPARATION 0.985 // meter
#define WHEEL_BASE 1.04 // meter
#define TURNING_RADIUS 0.080 // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define MAX_LINEAR_VELOCITY (WHEEL_RADIUS * 2 * 3.14159265359 * 3000 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY  -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY -MAX_ANGULAR_VELOCITY

//TODO: Define some global variables
using namespace std;

class erp42Core
{
public:
    erp42Core(ros::NodeHandle &nh):
    feedback_sub(nh.subscribe("output",100,&erp42Core::recvFeedback,this)),
    cmd_vel_sub(nh.subscribe("/cmd_vel",100,&erp42Core::commandVelocityCallback,this)),
    odom_pub(nh.advertise<nav_msgs::Odometry>("odom",10)),
    input_pub(nh.advertise<erp42_slam2d::ERP42_input>("input",100)),
    loop_rate(50)
    {
        init();
    }
    void init()
    {
        if(!nh.getParam("wheel_base", wheel_base_))
            wheel_base_ = 1.05;
         if(!nh.getParam("car_width", car_width_))
            car_width_ = 0.99;
         if(!nh.getParam("max_vel", max_vel_))
            max_vel_ = 5.55;
         if(!nh.getParam("min_vel", min_vel_))
            min_vel_ = -5.55;
         if(!nh.getParam("max_steering_angle", max_steering_angle_))
            max_steering_angle_ = 28.169;
         if(!nh.getParam("min_steering_angle", min_steering_angle_))
            min_steering_angle_ = -28.169;
        initOdom();
        prev_update_time_ = ros::Time::now();
    }

    void update()
    {
        publishDriveInformation();
        if(feedback_msg.mode == 1)
        {
            auto_mode();
        }
    }

    void spin()
    {
        while(ros::ok())
        {
            update();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void initOdom(void)
    {
        init_encoder = true;

        for (int index = 0; index < 3; index++)
        {
            odom_pose[index] = 0.0;
            odom_vel[index] = 0.0;
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
        ros::Duration step_time = time_now - prev_update_time_;
        prev_update_time_ = time_now;
//        ros::Time stamp_now = rosNow();

        calcOdometry(double(step_time.sec) + double(step_time.nsec)*1e-9);
        updateOdometry();
        odom.header.stamp = time_now;
        odom_pub.publish(odom);

        updateTF(odom_tf);
        odom_tf.header.stamp = time_now;
        tf_broadcaster_.sendTransform(odom_tf);
    }

    bool calcOdometry(double diff_time)

    {


        float* orientation;

        double wheel_s;

        double delta_s, theta ,delta_theta;

        double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]

        double step_time;

        wheel_s= 0;

        delta_s = delta_theta = theta = 0.0;

        v = w = 0.0;

        step_time = 0.0;

        step_time = diff_time;


        if(step_time == 0){

            std::cout<<"step time is 0;;"<<std::endl;

            return false;

        }

        if (isnan(wheel_s)){

            wheel_s = 0.0;
        }


        wheel_s = TICK2RAD * (feedback_msg.encoder - last_encoder); // OK

        last_encoder = feedback_msg.encoder;

        delta_s     = WHEEL_RADIUS * wheel_s; // OK
        v = delta_s / step_time;

        if(feedback_msg.steer_rad < 0)
        {
            delta_theta = (( (v / WHEEL_BASE) * tan(feedback_msg.steer_rad) ) * step_time) * 0.5454545454; // outside rotate
        }
        else if(feedback_msg.steer_rad > 0)
        {
            delta_theta = (( (v / WHEEL_BASE) * tan(feedback_msg.steer_rad) ) * step_time) * 0.72; // inside rotate
        }
        else if(feedback_msg.steer_rad == 0)
        {
            delta_theta = (( (v / WHEEL_BASE) * tan(feedback_msg.steer_rad) ) * step_time); // standard rotate
        }

//        if(delta_theta  < 0)
//        {
//            delta_theta = delta_theta + 6.283185307;
//        }

       // compute odometric pose

//        odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0)); //////// No.2

//        odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));

        odom_pose[2] += delta_theta;

        odom_pose[0] += delta_s * cos(odom_pose[2]); //////// No.2

        odom_pose[1] += delta_s * sin(odom_pose[2]);

        if(odom_pose[2]  > 6.283185307)
        {
            odom_pose[2] = odom_pose[2] - 6.283185307;
        }

//        if(odom_pose[2]  < -6.283185307)
//        {
//            odom_pose[2] = odom_pose[2] + 6.283185307;
//        }

        if(odom_pose[2]  < 0)
        {
            odom_pose[2] = odom_pose[2] + 6.283185307;
        }


//        std::cout << "odom_pose[0] : " << odom_pose[0] << std::endl;
//        std::cout << "odom_pose[1] : " << odom_pose[1] << std::endl;
//        std::cout << "odom_pose[2] : " << odom_pose[2] * (180.0 / 3.141592) << std::endl << endl;

//        std::cout << "odom.pose.pose.x: " << odom.pose.pose.position.x;l

// compute odometric instantaneouse velocity

        w = delta_theta / step_time;

        odom_vel[0] = v;

        odom_vel[1] = 0.0;

        odom_vel[2] = w;

        return true;

    }

    void updateOdometry(void)
    {
        odom.header.frame_id = odom_header_frame_id;
        odom.child_frame_id  = odom_child_frame_id;

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
             goal_velocity_from_cmd[0]  = cmd_vel_msg.linear.x;
             goal_velocity_from_cmd[1] = cmd_vel_msg.angular.z;
             std::cout << " i got messsage from cmdvelTest : " <<goal_velocity_from_cmd[0]<<std::endl;
    }

    void recvFeedback(const erp42_slam2d::ERP42_feedback &msg)
    {
        if(!feedback_flag)
        {
            feedback_msg = msg;
            feedback_flag = true;
        }
        last_encoder = feedback_msg.encoder;
        feedback_msg = msg;
    }
   void auto_mode()
    {
       if(feedback_msg.speed_mps <abs(goal_velocity_from_cmd[0]) && feedback_msg.speed_mps <0.5)
        input_msg.speed_mps = goal_velocity_from_cmd[0] > 0 ? 1.3 : -1.3;
       else {
           input_msg.speed_mps=goal_velocity_from_cmd[0];
       }
       //-----------------------------
        auto_brake();
        input_msg.mode=1;

        if(input_msg.speed_mps >=0){
            input_msg.gear=0;
            input_msg.speed_kph = (float) input_msg.speed_mps * (3600 / 1000);
    


	}
        else {
            input_msg.gear=2;
            input_msg.speed_kph = (float) -input_msg.speed_mps * (3600 / 1000);
	       
	}
       input_msg.steer_rad = goal_velocity_from_cmd[1];
        input_msg.steer_degree = input_msg.steer_rad *180 /3.141592;
        input_msg.Estop = 0;
        input_msg.alive = feedback_msg.alive;

        
        input_msg.brake = brake;
        input_pub.publish(input_msg);
    }

    void auto_brake()

    {
        //head to down

        if(feedback_msg.speed_mps > abs(goal_velocity_from_cmd[0]))
        {
       //brake----
            error = feedback_msg.speed_mps - goal_velocity_from_cmd[0];
            brake = 70 + std::round( 100 * (error) );
            if(brake >200) brake =200;
            std::cout << "-------------------------" <<std::endl;
            std::cout << "for now, erp is heading to downhill" <<std::endl;
            std::cout << "error : " << error <<endl;
            std::cout << "brake : "<< static_cast<float>(brake) <<endl;
            std::cout << "goal velocity : "<< static_cast<float>(goal_velocity_from_cmd[0]) <<endl;
            std::cout << "feedback_msg.speed_mps : "<< static_cast<float>(feedback_msg.speed_mps) <<endl;
            std::cout << "-------------------------" <<std::endl;
        }

        else if(feedback_msg.speed_mps < abs(goal_velocity_from_cmd[0]))
        {
            brake = 1;
            std::cout << "-------------------------" <<std::endl;
            std::cout << "goal velocity : "<< static_cast<float>(goal_velocity_from_cmd[0]) <<endl;
            std::cout << "feedback_msg.speed_mps : "<< static_cast<float>(feedback_msg.speed_mps) <<endl;
            std::cout << "-------------------------" <<std::endl;
        }
        else if(goal_velocity_from_cmd[0] == 0)
        {
            brake = 200;
        }
    }



private:
    /*Subscriber */
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber feedback_sub;
    /*Publisher */
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub;
    ros::Publisher input_pub;

    /*Transform Broadcaster */
    geometry_msgs::TransformStamped odom_tf;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /*Parameter*/
    erp42_slam2d::ERP42_feedback feedback_msg;
    erp42_slam2d::ERP42_input input_msg;
    float goal_velocity_from_cmd[2] = {0.0, 0.0};
    float odom_pose[3];
    double odom_vel[3];

    ros::Rate loop_rate;

    double wheel_base_;
    double car_width_;
    double max_vel_;
    double min_vel_;
    double max_steering_angle_;
    double min_steering_angle_;
    ros::NodeHandle nh;

    char odom_header_frame_id[10] = "odom";
    char odom_child_frame_id[15] = "base_footprint";
    int brake=1;
    double error;
    float last_encoder;
    bool feedback_flag = false;
    bool init_encoder = true;
    ros::Time prev_update_time_;    // phm
    ros::Time last_cmd_vel_time_;   // phm
    double cmd_vel_timeout_;        // phm
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erp42_core");
    ros::NodeHandle nh;
    erp42Core core(nh);
    core.spin();
    return 0;
}

