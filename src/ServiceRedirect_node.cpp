#include <ros/ros.h>
#include "elikos_roomba/ServiceRedirect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serviceredirect");
    ros::NodeHandle n;

    ServiceRedirect serviceredirect_(n);
    
    try
    {
        ros::spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[SERVICE REDIRECT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}