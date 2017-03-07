#include <ros/ros.h>
#include <std_srvs/Empty.h>

bool triggerTSturn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("Roomba does a 45-deg turn");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;
    
    // SERVICE: /topswitch_trigger
    ros::ServiceServer service = n.advertiseService("topswitch_trigger", triggerTSturn);
    
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        // do stuff
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}