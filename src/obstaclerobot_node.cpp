#include "elikos_roomba/obstaclerobot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstaclerobot");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    int robot_id;
    n_p.getParam("robot_id", robot_id);

    ObstacleRobot obstaclerobot_(n, robot_id);
    
    try
    {
        obstaclerobot_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[OBSTACLE ROBOT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}