#ifndef OBSTACLEROBOT_H
#define OBSTACLEROBOT_H

#include <ros/ros.h>
#include "elikos_roomba/robot.h"

// names
static const std::string OBSTACLEROBOT_TYPE = "OBSTACLE ROBOT";
// trajectory
static const double CIRC_TRAJECTORY_DIAMETER = 10.0;                                    //[m]
// find angular velocity
static const double CIRC_TRAJECTORY_CIRCUMFERENCE = CIRC_TRAJECTORY_DIAMETER*3.1415;    //[m]
static const double CIRC_TRAJECTORY_DURATION = CIRC_TRAJECTORY_DIAMETER/FORWARD_SPEED;  //[m]/[m/s]=[s]
static const float CIRC_TURN_SPEED = 360.0f*DEG_TO_RAD/CIRC_TRAJECTORY_DURATION;         //[deg]*[rad/deg]/[s]=[rad/s]

class ObstacleRobot : public Robot
{
    private:
        // obstacle robot states
        enum ObstacleRobotState {
            INACTIVE,
            CIRCULAR
        };
        ObstacleRobotState robotState_;

    protected:
        void update();
        void updateState();

        // obstacle robot state changes
        void changeRobotStateTo(ObstacleRobotState newRobotState);
        bool isRobotState(ObstacleRobotState cmpRobotState);

        // global state changes
        void activateRobot();
        void deactivateRobot();
    
    public:
        ObstacleRobot(ros::NodeHandle& n);
        ~ObstacleRobot();
        void spin();
        void spinOnce();
};

#endif  // ELIKOS_ROOMBA_OBSTACLEROBOT_H