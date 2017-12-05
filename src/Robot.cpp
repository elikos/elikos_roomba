/**
 * \file Robot.cpp
 * \brief Robot class implementation
 * \author christophebedard
 */

#include "elikos_roomba/Robot.h"

Robot::Robot(ros::NodeHandle& n, std::string botType, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string model_option)
    : robotType_(botType),
      r_id_(r_id),
      MovingObject(n, getRobotNamespace(botType, r_id), initial_pos, initial_yaw, model_option)
{
    // setup publishers
    robotState_pub_ = n.advertise<std_msgs::String>(ns_ + "/" + ROBOTSTATE_TOPIC_NAME, 10);

    // setup services
    activate_srv_ = n.advertiseService(ns_ + "/" + ACTIVATE_SERVICE_NAME, &Robot::activateCallback, this);
    deactivate_srv_ = n.advertiseService(ns_ + "/" + DEACTIVATE_SERVICE_NAME, &Robot::deactivateCallback, this);
    toglActivate_srv_ = n.advertiseService(ns_ + "/" + TOGGLEACT_SERVICE_NAME, &Robot::toglActivateCallback, this);

    ROS_INFO_STREAM_ROBOT("Robot initialization done (inactive)");
}

Robot::~Robot() {
  ROS_INFO_STREAM_ROBOT("Robot base destruct robot sequence initiated");
  // add other relevant stuff
}

std::string Robot::getRobotType() const {
    return robotType_;
}

/*===========================
 * Other utilities
 *===========================*/

void Robot::ROS_INFO_STREAM_ROBOT(std::string message) {
    ROS_INFO_STREAM("[" << ns_ << "] " << message);
}

std::string Robot::getRobotNamespace(std::string robotType, int robotId) {
    return robotType + "robot" + std::to_string(robotId);
}

/*===========================
 * Global state
 *===========================*/

void Robot::activateRobot() {
    ROS_INFO_STREAM_ROBOT("Robot activated");
    isActive_ = true;
}

void Robot::deactivateRobot() {
    ROS_INFO_STREAM_ROBOT("Robot deactivated");
    isActive_ = false;
}

/*===========================
 * Callbacks
 *===========================*/

bool Robot::activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    if (!isActive_) { activateRobot(); }
    return true;
}

bool Robot::deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    if (isActive_) { deactivateRobot(); }
    return true;
}

bool Robot::toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    isActive_ ? deactivateRobot() : activateRobot();
    return true;
}

/*===========================
 * Update
 *===========================*/

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
    // update position before cmd_vel
    MovingObject::update();

    // ghetto way to only publish cmdvel if robot is active
    if (isActive_) { MovingObject::publishCmdVel(); }

    publishRobotState();
}
