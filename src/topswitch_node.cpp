#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

/**
 * Make sure the wiringPi library is installed: http://wiringpi.com/download-and-install/
 */
//#include <wiringPi.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topswitch");
    ros::NodeHandle n;
    
    /**
    * TOPIC:
    *    - topswitch_state
    */
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("topswitch_state", 1000);
    
    ros::Rate loop_rate(10);
    
    /**
    * Initialize interrupt for switch
    */
    //wiringPiSetup();
    //bool ts_state
    
    while (ros::ok())
    {
        /**
        * message
        */
        std_msgs::String msg;
        
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        
        ROS_INFO("%s", msg.data.c_str());
        
        /**
        * publish()
        */
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    return 0;
}