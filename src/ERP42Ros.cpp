#include "ros/ros.h"

#include "erp42_ros/ERP42_input.h"
#include "erp42_ros/ERP42_feedback.h"

namespace
{
struct upper_to_pcu
{
    unsigned char stx[3];
    unsigned char AorM;
    unsigned char EStop;
    unsigned char gear;
    union {
        unsigned char __speed[2];
        unsigned short speed;
    };
    union {
        unsigned char __steer[2];
        short steer;
    };
    unsigned char brake;
    unsigned char alive;
    unsigned char ext[2];
};

struct pcu_to_upper
{
    unsigned char stx[3];
    unsigned char AorM;
    unsigned char EStop;
    unsigned char gear;
    union {
        unsigned char __speed[2];
        unsigned short speed;
    };
    union {
        unsigned char __steer[2];
        short steer;
    };
    unsigned char brake;
    union {
        unsigned char __enc[4];
        int enc;
    };
    unsigned char alive;
    unsigned char ext[2];
};

enum packet_size : unsigned int
{
    SIZE_P2U = sizeof(pcu_to_upper),
    SIZE_U2P = sizeof(upper_to_pcu),
};
} // namespace

class ERP42Interface
{
public:
    ERP42Interface(ros::NodeHandle &n, const char *device_name)
        : pub(n.advertise<erp42_ros::ERP42_feedback>("output", 100)),
          sub(n.subscribe("input", 100, &ERP42Interface::OnInputMsgRecv, this)),
          loop_rate(50)
    {
        // TODO :
        // open serial device
        // if open device fails, throw.
        // there is no catch for this throw, so this process will be exit with error.
    }
    ~ERP42Interface()
    {
        // TODO :
        // close the device
    }
    void OnInputMsgRecv(const erp42_ros::ERP42_input &msg)
    {
        input_msg = msg;
        // this is debug message. if you want to see if this node receives topic,
        // uncomment following line
        // ROS_INFO("ERP42_input message received!\n");
    }
    void PublishFeedback()
    {
        pub.publish(feedback_msg);
    }

    // update() function will be called every 20ms.(50Hz)
    // this is control frequency
    void update()
    {
        // TODO :
        // Receive feedback message from RS232
        // and store it to the this->u2p

        // TODO :
        // if there was incomming message from RS232,
        // copy the message to the this->feedback_msg properly

        // After Copy, Publish to the ROS
        PublishFeedback();

        // TODO :
        // make packet on p2u from latest ERP42_input

        // TODO :
        // send the packet via RS232
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

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Rate loop_rate;

    upper_to_pcu u2p;
    pcu_to_upper p2u;

    erp42_ros::ERP42_feedback feedback_msg;
    erp42_ros::ERP42_input input_msg;

    // TODO :
    // Any variable you need can be created here.
};

void print_usage()
{
    ROS_INFO("Need more arguments\n"
             "Usage: ERP42Ros [serial device path]\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ERP42Ros");
    if (argc < 2)
    {
        print_usage();
        return -1;
    }
    // now argv[1] is devname

    ros::NodeHandle n;
    ERP42Interface erp42(n, argv[1]);
    erp42.spin();
    return 0;
}