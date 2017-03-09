#include "elikos_roomba/robot.h"

Robot::Robot(ros::NodeHandle& n)
    : n_(n)
{
    //_robotType = botType;

    // Setup publishers
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
}

Robot::~Robot() {
  ROS_INFO("[ROBOT] Destruct robot sequence initiated.");
  // add other relevant stuff
}

void Robot::update() {
    // to do

    // check timers
}

geometry_msgs::Twist Robot::getCmdVelMsg(float lin_x, float ang_z) {
    geometry_msgs::Twist cmdVel_msg;
    cmdVel_msg.linear.x = lin_x;
    cmdVel_msg.angular.z = ang_z;
    
    return cmdVel_msg;
}

void Robot::publishCmdVel(geometry_msgs::Twist cmdVel_msg) {
    cmd_vel_pub_.publish(cmdVel_msg);
}


// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(10);

    Robot robot_(n);

    geometry_msgs::Twist ms = robot_.getCmdVelMsg(0.0f, 2.5f);
    
    while (ros::ok())
    {
        // do stuff
        robot_.publishCmdVel(ms);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}