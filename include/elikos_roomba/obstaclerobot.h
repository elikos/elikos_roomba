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
static const float CIRC_TURN_SPEED = 360.0f*DEG_TO_RAD/CIRC_TRAJECTORY_DURATION;        //[deg]*[rad/deg]/[s]=[rad/s]

class ObstacleRobot : public Robot
{
    private:
        /*===========================
         * Obstacle robot state
         *===========================*/
        enum ObstacleRobotState {
            INACTIVE,
            CIRCULAR
        };
        ObstacleRobotState robotState_;

    protected:
        /*===========================
         * Update
         *===========================*/
        /*
         * Update robot state; called every spinOnce()
         */
        void update();

        /*
         * Update ground robot message based on current state
         */
        void updateState();

        /*
         * ROS spin once, called on every loop
         */
        void spinOnce();

        /*===========================
         * Global state
         *===========================*/
        /*
         * Activate global robot state
         */
        void activateRobot();

        /*
         * Deactivate global robot state
         */
        void deactivateRobot();

        /*===========================
         * Obstacle robot state changes
         *===========================*/
        /*
         * Change robot state to new given state
         */
        void changeRobotStateTo(ObstacleRobotState newRobotState);

        /*
         * Compare current robot state to given state
         */
        bool isRobotState(ObstacleRobotState cmpRobotState);
    
    public:
        ObstacleRobot(ros::NodeHandle& n, int r_id);
        ~ObstacleRobot();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();
};

#endif  // ELIKOS_ROOMBA_OBSTACLEROBOT_H