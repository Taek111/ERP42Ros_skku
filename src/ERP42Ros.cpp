#include "ros/ros.h"
#include <cmath>
#include "erp42_ros/ERP42_input.h"
#include "erp42_ros/ERP42_feedback.h"
#include "SerialPort.h"
using namespace std;
namespace
{
struct upper_to_pcu
{
    unsigned char stx[3] = {0x53, 0x54, 0x58};
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
    unsigned char ext[2] = {0x0d, 0x0a};
};

struct pcu_to_upper
{
    unsigned char stx[3] = {0x53, 0x54, 0x58};
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
    unsigned char ext[2] = {0x0d, 0x0a};
};

enum packet_size : unsigned int
{
    SIZE_P2U = sizeof(pcu_to_upper),
    SIZE_U2P = sizeof(upper_to_pcu),
};
}; // namespace

class ERP42Interface
{
public:
    ERP42Interface(ros::NodeHandle &n, const char *device_name)
        : pub(n.advertise<erp42_ros::ERP42_feedback>("output", 100)),
          sub(n.subscribe("input", 100, &ERP42Interface::OnInputMsgRecv, this)),
          RS232(device_name)
    {
        // TODO :
        // open serial device
        // if open device fails, throw.
        // there is no catch for this throw, so this process will be exit with error.
        // this TODO is not cleared. 
        
        // trouble: double initialization. 
        // trouble: not using device_name forwarded.
        RS232 = SerialPort("/dev/ttyUSB0");
    }
    ~ERP42Interface()
    {
        // TODO :
        // close the device
         RS232.Close();
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
        RS232.ReadPacket(readBuffer);
        p2uPacket = reinterpret_cast<unsigned char*>(const_cast<char*>(readBuffer.c_str()));
        p2u.AorM = p2uPacket[0];
        p2u.EStop = p2uPacket[1];
        p2u.gear = p2uPacket[2];
        p2u.__speed[0] = p2uPacket[3];
        p2u.__speed[1] = p2uPacket[4];
        p2u.__steer[0] = p2uPacket[5];
        p2u.__steer[1] = p2uPacket[6];
        p2u.brake = p2uPacket[7];
        p2u.__enc[0] = p2uPacket[8];
        p2u.__enc[1] = p2uPacket[9];        
        p2u.__enc[2] = p2uPacket[10];  
        p2u.__enc[3] = p2uPacket[11];  
        p2u.alive = p2uPacket[12];
        // TODO :
        // if there was incomming message from RS232,
        // copy the message to the this->feedback_msg properly
        feedback_msg.mode = (p2u.AorM == 0x01)? (int16_t)1 : (int16_t)0; 
        feedback_msg.Estop = (p2u.EStop == 0x01)? true : false;
        feedback_msg.gear =  (int16_t)p2u.gear;
        feedback_msg.speed_mps = (double) p2u.speed / 36;
        feedback_msg.speed_kph = (double) p2u.speed / 10;
        feedback_msg.steer_rad = (double) (M_PI / 180) * p2u.steer / 71;
        feedback_msg.steer_degree = (double) p2u.steer / 71;
        feedback_msg.brake = (int16_t) p2u.brake;
        feedback_msg.encoder = p2u.enc;

        // After Copy, Publish to the ROS
        PublishFeedback();

        // TODO :
        // make packet on u2p from latest ERP42_input
        u2p.AorM = (input_msg.mode == 1) ? 0x01 : 0x00;
        u2p.EStop = (input_msg.Estop == true) ? 0x01: 0x00;
        u2p.gear = (unsigned char)input_msg.gear;
        u2p.speed = (unsigned short)(input_msg.speed_kph * 10);
        u2p.steer = (short)(input_msg.steer_degree * 71);
        u2p.brake = (unsigned char)input_msg.brake;
        u2p.alive = p2u.alive;
        // TODO :
        // send the packet via RS232
        
        u2pPacket[0] = 0x53;
        u2pPacket[1] = 0x54;
        u2pPacket[2] = 0x58;
        u2pPacket[3] = u2p.AorM;
        u2pPacket[4] = u2p.EStop;
        u2pPacket[5] = u2p.gear;
        u2pPacket[6] = u2p.__speed[0];
        u2pPacket[7] = u2p.__speed[1];
        u2pPacket[8] = u2p.__steer[0];
        u2pPacket[9] = u2p.__steer[1];
        u2pPacket[10] = u2p.brake;
        u2pPacket[11] = u2p.alive;
        u2pPacket[12] = 0x0d;
        u2pPacket[13] = 0x0a;   
        writeBuffer = reinterpret_cast<char*>(u2pPacket);
        RS232.Write(writeBuffer);
        
    }

    void spin()
    {
        while (ros::ok())
        {
            update();
            ros::spinOnce();
            //loop_rate.sleep();
        }
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;

    upper_to_pcu u2p;
    pcu_to_upper p2u;

    erp42_ros::ERP42_feedback feedback_msg;
    erp42_ros::ERP42_input input_msg;

    // TODO :
    // Any variable you need can be created here.
    unsigned char* p2uPacket;
    string readBuffer; 
    unsigned char u2pPacket[14];
    string writeBuffer;
    
    SerialPort RS232;
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
