#include "elikos_roomba/GroundRobot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    double init_pos_x, init_pos_y, init_pos_z, init_yaw;
    int robot_id;
    std::string robot_color;
    n_p.getParam("init_pos_x", init_pos_x);
    n_p.getParam("init_pos_y", init_pos_y);
    n_p.getParam("init_pos_z", init_pos_z);
    n_p.getParam("init_yaw", init_yaw);
    n_p.getParam("robot_id", robot_id);
    n_p.getParam("robot_color", robot_color);

    GroundRobot groundrobot_(n, robot_id, tf::Vector3(init_pos_x, init_pos_y, init_pos_z), init_yaw, robot_color);
    
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