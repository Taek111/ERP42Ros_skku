#include "ros/ros.h"
#include <cmath>
#include <time.h>
#include "erp42_ros/ERP42_input.h"
#include "erp42_ros/ERP42_feedback.h"
#include "SerialPort.h"
#include<iostream>
#include<fstream>
#include<string>

using namespace std;
namespace
{
struct upper_to_pcu
{
    unsigned char stx[3] = {0x53, 0x54, 0x58};
    unsigned char AorM = {0x01}; //Auto mode 
    unsigned char EStop = {0x00}; //E-STOP Off
    unsigned char gear = 0x01; //neutral
    union {
        unsigned char __speed[2]; //speed = 0 kph
        unsigned short speed = 0;
    };
    union {
        unsigned char __steer[2]; // steer = 0
        short steer = 0;
    };
    unsigned char brake = 0x3C; //30% braking(60) 
    unsigned char alive ;
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
class ringbuf
{
private:
    unsigned char *_buf;
    size_t _cursor;
    const size_t _size;

public:
    ringbuf(size_t size) : _buf(new unsigned char[size]), _cursor(0), _size(size)
    {
        unsigned char*iter = _buf;
        const  unsigned char*end = _buf + size;
        while (iter != end)
        {
            *iter = 0;
            ++iter;
        }
    }
    ~ringbuf()
    {
        delete[] _buf;
    }
    void reset() { _cursor = 0; }
    size_t size() { return _size; }
    const  unsigned char&at(size_t index)
    {
        return _buf[(index + _cursor) % _size];
    }
    unsigned char*cursor()
    {
        return &_buf[_cursor];
    }
    void next(size_t offset)
    {
        _cursor = (_cursor + offset) % _size;
    }
    const  unsigned char&operator[](size_t index)
    {
        return at(index);
    }
    void operator++()
    {
        return next(1);
    }
    void push(const  unsigned char&data)
    {
        *cursor() = data;
        next(1);
    }
    void copy(unsigned char *dst, size_t size)
    {
        size_t i = 0;
        unsigned char*iter = (unsigned char*)dst;
        const  unsigned char*end = iter + size;
        while (iter != end)
        {
            *iter = at(i);
            ++iter;
            ++i;
            if (i == size)
                break;
        }
    }
};
bool is_valid_P2U(ringbuf &buf)
{
    return (buf[0] == 0x0A) &&
           (buf[1] == 0x0D) &&
           (buf[15] == 0x58) &&
           (buf[16] == 0x54) &&
           (buf[17] == 0x53);
}
void ntoh_packet(unsigned char * packet, int size)
{
    unsigned char temp;
    for (int i=0; i<size/2; i++)
    {
        temp = packet[i];
        packet[i] = packet[size-i-1];
        packet[size-i-1] = temp;
    }
}

string p2ulog_file;
string u2plog_file;

void make_logfile()
{
	time_t timer = time(NULL);
	struct tm *t = localtime(&timer);
	string log_path;
	string file_num = to_string(t->tm_mon+1) +  "." + to_string(t->tm_mday);
    if(ros::param::get("log_path", log_path))
	{	
		p2ulog_file = log_path + "p2ulog(" + file_num + ").txt";
		u2plog_file = log_path + "u2plog(" + file_num + ").txt";
	}
	else
	cout << "get log_path failed!" << endl;
}

} // namespace


class ERP42Interface
{
public:
    ERP42Interface(ros::NodeHandle &n, const char *device_name)
        : pub(n.advertise<erp42_ros::ERP42_feedback>("output", 100)),
          sub(n.subscribe("input", 100, &ERP42Interface::OnInputMsgRecv, this)),
          loop_rate(50), RS232(device_name), p2ulog(p2ulog_file.c_str(), ios::app),
          u2plog(u2plog_file.c_str(), ios::app), p2ubuffer(ringbuf(18))
    {
        // open serial device
        // if open device fails, throw.
        // there is no catch for this throw, so this process will be exit with error.        
    }
    ~ERP42Interface()
    {
        // close the device
         RS232.Close();
         p2ulog.close();
         u2plog.close();
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
        // Receive feedback message from RS232
        // and store it to the this->u2p
        
		unsigned char*p2uPacket = (unsigned char *)&p2u;
      	readPacket(p2uPacket, 18);
        if(p2ulog.is_open()){
            p2ulog << time(NULL) <<",";                             //timestamp
            p2ulog << static_cast<int> (p2u.AorM) <<",";   //Auto/Manual
            p2ulog << static_cast<int>(p2u.EStop) <<",";    //E-STOP
            p2ulog <<static_cast<int>(p2u.gear) <<",";        //Gear
            p2ulog << p2u.speed <<",";                                //Speed
            p2ulog << p2u.steer <<",";                                  //Steer
            p2ulog << static_cast<int>(p2u.brake) <<",";    //Brake
            p2ulog << p2u.enc <<",";                                    //Encoder
            p2ulog <<  static_cast<int>(p2u.alive) << endl; //Alive
        }
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

        // make packet on u2p from latest ERP42_input
        u2p.AorM = (input_msg.mode == 1) ? 0x01 : 0x00;
        u2p.EStop = (input_msg.Estop == true) ? 0x01: 0x00;
        u2p.gear = (unsigned char)input_msg.gear;
        u2p.speed = (unsigned short)(input_msg.speed_kph * 10);
        u2p.steer = (short)(input_msg.steer_degree * 71);
        u2p.brake = (unsigned char)input_msg.brake;
        u2p.alive = p2u.alive;

        // send the packet via RS232
        unsigned char*u2pPacket = (unsigned char *)&u2pPacket;
        u2pPacket[3] = u2p.AorM;
        u2pPacket[4] = u2p.EStop;
        u2pPacket[5] = u2p.gear;
        u2pPacket[6] = u2p.__speed[0];
        u2pPacket[7] = u2p.__speed[1];
        u2pPacket[8] = u2p.__steer[0];
        u2pPacket[9] = u2p.__steer[1];
        u2pPacket[10] = u2p.brake;
        u2pPacket[11] = u2p.alive;
        RS232.Write(u2pPacket, 14);
         if(u2plog.is_open()){
            u2plog << time(NULL) <<",";                             //timestamp
            u2plog << static_cast<int> (u2p.AorM) <<",";   //Auto/Manual
            u2plog << static_cast<int>(u2p.EStop) <<",";    //E-STOP
            u2plog <<static_cast<int>(u2p.gear) <<",";        //Gear
            u2plog << u2p.speed <<",";                                //Speed
            u2plog << u2p.steer <<",";                                  //Steer
            u2plog << static_cast<int>(u2p.brake) <<",";    //Brake        
            u2plog <<  static_cast<int>(u2p.alive) << endl; //Alive
        }       
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

    void readPacket(unsigned char* packetBuffer, int size)
    {
        RS232.Read(packetBuffer, 1);
        p2ubuffer.push(packetBuffer[0]);
        while(!is_valid_P2U(p2ubuffer))
        {
            RS232.Read(packetBuffer, 1);
            p2ubuffer.push(packetBuffer[0]);
        }
        p2ubuffer.copy(packetBuffer, size);
        ntoh_packet(packetBuffer, size);
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Rate loop_rate;
    upper_to_pcu u2p;
    pcu_to_upper p2u;
    erp42_ros::ERP42_feedback feedback_msg;
    erp42_ros::ERP42_input input_msg;
    SerialPort RS232;
    unsigned char u2pPacket[14];
    unsigned char testwrite[18];
    ofstream u2plog;
    ofstream p2ulog;
    ringbuf p2ubuffer;
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
    make_logfile();
    ERP42Interface erp42(n, argv[1]);
    erp42.spin();
    return 0;
}
