#include "elikos_roomba/robot.h"

Robot::Robot(ros::NodeHandle& n)
    : n_(n),
      is_running_slowly_(false)
{
    loop_hz_ = 10.0;

    //_robotType = botType;

    // Setup publishers
    ROS_INFO_STREAM("Setting up cmd_vel publisher..");
    cmdVel_pub_ = n.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC_NAME, CMDVEL_TOPIC_QUEUESIZE);
}

Robot::~Robot() {
  ROS_INFO_STREAM("[ROBOT] Destruct robot sequence initiated.");
  // add other relevant stuff
}

void Robot::publishCmdVel() {
    cmdVel_pub_.publish(cmdVel_msg_);
}

void Robot::publishCmdVel(geometry_msgs::Twist msg_) {
    cmdVel_pub_.publish(msg_);
}

geometry_msgs::Twist Robot::getCmdVelMsg(float lin_x, float ang_z) {
    geometry_msgs::Twist cmdVel_msg;
    cmdVel_msg.linear.x = lin_x;
    cmdVel_msg.angular.z = ang_z;
    
    return cmdVel_msg;
}

void Robot::update() {
    ROS_INFO_STREAM("[ROBOT] update");

    publishCmdVel();
}

/*void Robot::spinOnce()
{
  update();
  ros::spinOnce();
}

void Robot::spin()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[ROBOT] Loop running slowly.");
    }
  }
}*/

// ---------------------------

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;

    Robot robot_(n);

    //geometry_msgs::Twist ms = robot_.getCmdVelMsg(0.0f, 2.5f);
    //robot_.publishCmdVel(ms);
    
    try
    {
        //robot_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ROBOT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}*/
