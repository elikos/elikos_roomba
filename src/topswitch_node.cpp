#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <wiringPi.h> // Make sure the wiringPi library is installed

#define TOPSWITCH_PIN   17

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topswitch");
    ros::NodeHandle n;
    
    /**
    * TOPIC:
    *    - topswitch_state
    */
    ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("topswitch_state", 1000);
    
    ros::Rate loop_rate(10);
    
    // Initialize interrupt for switch
    wiringPiSetupGpio(); //broadcom pin numbers
    pinMode(TOPSWITCH_PIN, INPUT);
    pullUpDnControl(TOPSWITCH_PIN, PUD_UP);
    
    bool isTopSwitchTouched;
    while (ros::ok())
    {
        // get topswitch state
        isTopSwitchTouched = !digitalRead(TOPSWITCH_PIN);
        if (isTopSwitchTouched) {
            ROS_INFO("SIGNAL");
        } else {
            ROS_INFO("NO SIGNAL");
        }
        
        // message
        std_msgs::Bool msg;
        msg.data = isTopSwitchTouched;
        
        // publish()
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    return 0;
}