#include "elikos_roomba/obstaclerobot.h"

ObstacleRobot::ObstacleRobot(ros::NodeHandle& n, int r_id)
    : Robot(n, OBSTACLEROBOT_TYPE, r_id)
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
  ros::Rate rate(loop_hz_);

  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[OBSTACLE ROBOT] Loop running slowly.");
    }
  }
}

// ---------------------------
/*
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
*/