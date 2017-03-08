#include "elikos_roomba/robot.h"

Robot::Robot(ros::NodeHandle& n, RobotType botType) {
    _n = n;
    _robotType = botType;

    // Setup publishers
    _cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 50)
}

Robot::~Robot() {
  ROS_INFO("[ROBOT] Destruct robot sequence initiated.");
  // add other relevant stuff
}

void Robot::update() {
    // to do

    // check timers
}

geometry_msgs::Twist Robot::getCmdVel(float lin_x, float ang_z) {
    geometry_msgs::Twist cmdVel_msg;
    cmdVel_msg.linear.x = lin_x;
    cmdVel_msg.angular.z = ang_z;
    
    return cmdVel_msg;
}

Robot::publishCmdVel(geometry_msgs::Twist cmdVel_msg) {
    _cmd_vel_pub.publish(cmdVel_msg);
}


// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(10);

    Robot _robot(n);

    _robot.publishCmdVel(_robot.getCmdVelMsg(float 0.2, float 0.1));
    
    while (ros::ok())
    {
        // do stuff

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}