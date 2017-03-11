#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <wiringPi.h> // Make sure the wiringPi library is installed
#include <std_srvs/Empty.h>

//broadcom pin number
#define TOPSWITCH_PIN 17

bool isTopSwitchActivated = false;
bool wasTopSwitchActivated = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topswitch");
    ros::NodeHandle n;
    
    // TOPIC: /topswitch_state
    ros::Publisher topswitch_pub = n.advertise<std_msgs::Bool>("topswitch_state", 1000);
    
    // SERVICE
    ros::ServiceClient topswitch_srv_client = n.serviceClient<std_srvs::Empty>("topswitch_trigger");
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