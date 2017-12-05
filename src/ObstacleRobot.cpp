/**
 * \file ObstacleRobot.cpp
 * \brief ObstacleRobot class implementation
 * \author christophebedard
 */

#include "elikos_roomba/ObstacleRobot.h"

ObstacleRobot::ObstacleRobot(ros::NodeHandle& n, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string height)
    : Robot(n, OBSTACLEROBOT_TYPE, r_id, initial_pos, initial_yaw, height)
{
    // initial state
    changeRobotStateTo(INACTIVE);

    //ROS_INFO_STREAM_ROBOT("Parent initialization done");
}

ObstacleRobot::~ObstacleRobot() {
    Robot::ROS_INFO_STREAM_ROBOT("Destruct obstacle robot sequence initiated.");
    // add other relevant stuff
}

/*===========================
 * Obstacle robot state changes
 *===========================*/

void ObstacleRobot::changeRobotStateTo(ObstacleRobotState newRobotState) {
    robotState_ = newRobotState;

}

bool ObstacleRobot::isRobotState(ObstacleRobotState cmpRobotState) {
    return robotState_ == cmpRobotState;
}

/*===========================
 * Global state
 *===========================*/

void ObstacleRobot::activateRobot() {
    //ROS_INFO_STREAM_ROBOT("Parent robot activated");
    changeRobotStateTo(CIRCULAR);
    Robot::activateRobot();
}

void ObstacleRobot::deactivateRobot() {
    //ROS_INFO_STREAM_ROBOT("Parent robot deactivated");
    changeRobotStateTo(INACTIVE);
    Robot::deactivateRobot();
}

/*===========================
 * Update
 *===========================*/

void ObstacleRobot::updateState() {
    // check reset state
    if (isReset_) {
        deactivateRobot();
    }

    // set cmd_vel msg according to state
    switch ( robotState_ ) {
        case INACTIVE:
            // nothing
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            robotState_msg_.data = "INACTIVE";
            break;
        case CIRCULAR:
            cmdVel_msg_ = Robot::getCmdVelMsg(FORWARD_SPEED, CIRC_TURN_SPEED*ROTATE_CW);
            robotState_msg_.data = "CIRCULAR";
            break;
        default:
            // default behaviour
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            robotState_msg_.data = "INACTIVE";
            break;
    }
}

void ObstacleRobot::update() {
    //Robot::ROS_INFO_STREAM_ROBOT("update");
    updateState();
    Robot::update();
}

void ObstacleRobot::spinOnce()
{
  ObstacleRobot::update();
  ros::spinOnce();
}

void ObstacleRobot::spin()
{
  ros::Rate rate(LOOP_RATE);

  while (ros::ok())
  {
    spinOnce();

    if (!rate.sleep())
    {
      ROS_WARN("[OBSTACLE ROBOT] Loop running slowly.");
    }
  }
}
