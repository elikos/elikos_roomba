#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <wiringPi.h> // Make sure the wiringPi library is installed
#include <std_srvs/Empty.h>

// broadcom pin number
// see BCM 17: https://pinout.xyz/pinout/wiringpi
#define TOPSWITCH_PIN 17
// connect switch to TOPSWITCH_PIN and a ground pin

static const std::string TOPSWITCHSTATE_TOPIC_NAME = "topswitch_state";
static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";

bool isTopSwitchActivated = false;
bool wasTopSwitchActivated = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topswitch");
    ros::NodeHandle n;
    
    // setup publisher
    ros::Publisher topswitch_pub = n.advertise<std_msgs::Bool>(TOPSWITCHSTATE_TOPIC_NAME, 1000);
    
    // setup service
    ros::ServiceClient topswitch_srv_client = n.serviceClient<std_srvs::Empty>(TOPSWITCH_SERVICE_NAME);
    std_srvs::Empty srv;
    
    ros::Rate loop_rate(25);
    
    // setup wiringPi
    if (wiringPiSetupGpio() < 0) {
        ROS_ERROR("wiringPi: cannot setup wiringPi GPIO");
        return -1;
    }
    // setup pin (input with pull-up resistor)
    pinMode(TOPSWITCH_PIN, INPUT);
    pullUpDnControl(TOPSWITCH_PIN, PUD_UP);
    
    while (ros::ok())
    {
        // get topswitch state and compare with previous state
        isTopSwitchActivated = !digitalRead(TOPSWITCH_PIN);
        if (isTopSwitchActivated && !wasTopSwitchActivated) {
            // rising edge (ghetto style)
            ROS_INFO_STREAM("[TOPSWITCH] Rising edge");
            // call topswitch service
            topswitch_srv_client.call(srv.request, srv.response);
        }
        wasTopSwitchActivated = isTopSwitchActivated;
        
        // message
        std_msgs::Bool msg;
        msg.data = isTopSwitchActivated;
        
        // publish
        topswitch_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}