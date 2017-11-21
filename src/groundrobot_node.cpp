#include "elikos_roomba/groundrobot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    int robot_id;
    n_p.getParam("robot_id", robot_id);

    GroundRobot groundrobot_(n, robot_id);
    
    try
    {
        groundrobot_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[GROUND ROBOT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}