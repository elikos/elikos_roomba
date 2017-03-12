#include "elikos_roomba/robot.h"

Robot::Robot(ros::NodeHandle& n, std::string botType)
    : n_(n),
      is_running_slowly_(false),
      robotType_(botType)
{
    loop_hz_ = LOOP_RATE;

    // setup publishers
    cmdVel_pub_ = n.advertise<geometry_msgs::Twist>(CMDVEL_TOPIC_NAME, CMDVEL_TOPIC_QUEUESIZE);
    robotState_pub_ = n.advertise<std_msgs::String>(ROBOTSTATE_TOPIC_NAME, ROBOTSTATE_TOPIC_QUEUESIZE);

    // setup services
    activate_srv_ = n.advertiseService(ACTIVATE_SERVICE_NAME, &Robot::activateCallback, this);
    deactivate_srv_ = n.advertiseService(DEACTIVATE_SERVICE_NAME, &Robot::deactivateCallback, this);
    toglActivate_srv_ = n.advertiseService(TOGGLEACT_SERVICE_NAME, &Robot::toglActivateCallback, this);

    // initial active/inactive state
    //deactivateRobot();

    ROS_INFO_STREAM_ROBOT("Base initialization done");
}

Robot::~Robot() {
  ROS_INFO_STREAM_ROBOT("Robot base destruct robot sequence initiated");
  // add other relevant stuff
}

void Robot::ROS_INFO_STREAM_ROBOT(std::string message) {
    ROS_INFO_STREAM("[" << robotType_ << "] " << message);
}

void Robot::activateRobot() {
    ROS_INFO_STREAM_ROBOT("Base robot activated");
    isActive_ = true;
}
void Robot::deactivateRobot() {
    ROS_INFO_STREAM_ROBOT("Base robot deactivated");
    isActive_ = false;
}

bool Robot::activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    activateRobot();
    return true;
}
bool Robot::deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    deactivateRobot();
    return true;
}
bool Robot::toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    isActive_ ? deactivateRobot() : activateRobot();
    return true;
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

void Robot::publishRobotState() {
    robotState_pub_.publish(robotState_msg_);
}

void Robot::update() {
    //ROS_INFO_STREAM_ROBOT("base update");
    // GHETTO way to only publish cmdvel if robot is active
    if (isActive_) { publishCmdVel(); }

    publishRobotState();
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
