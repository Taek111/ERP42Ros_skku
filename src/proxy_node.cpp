
#include <erp42_ros/target.h>
#include <erp42_ros/ERP42_input.h>
#include <erp42_ros/ERP42_feedback.h>
#include <proxy/Proxy.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "proxy_node");

    uint16_t local_port, remote_port;
    std::string remote_host;

    ROSParamHelper p_helper;
    local_port = p_helper.getParam<int>("local_port");
    remote_port = p_helper.getParam<int>("remote_port");
    remote_host = p_helper.getParam<std::string>("remote_host");

    Proxy prx(local_port, remote_host, remote_port);

    prx.addPubFactory<erp42_ros::target>();
    prx.addPubFactory<erp42_ros::ERP42_feedback>();
    prx.addPubFactory<erp42_ros::ERP42_input>();
    auto sub0 = prx.getSubscriber<erp42_ros::target>("target");
    auto sub1 = prx.getSubscriber<erp42_ros::ERP42_feedback>("feedback");
    auto sub2 = prx.getSubscriber<erp42_ros::ERP42_input>("input");

    prx.spin();
    return 0;
}