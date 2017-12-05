#include "elikos_roomba/ObstacleRobot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstaclerobot");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    double init_pos_x, init_pos_y, init_pos_z, init_yaw;
    int robot_id;
    std::string robot_height;
    n_p.getParam("init_pos_x", init_pos_x);
    n_p.getParam("init_pos_y", init_pos_y);
    n_p.getParam("init_pos_z", init_pos_z);
    n_p.getParam("init_yaw", init_yaw);
    n_p.getParam("robot_id", robot_id);
    n_p.getParam("robot_height", robot_height);

    ObstacleRobot obstaclerobot_(n, robot_id, tf::Vector3(init_pos_x, init_pos_y, init_pos_z), init_yaw, robot_height);
    
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