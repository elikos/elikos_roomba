#ifndef OBSTACLEROBOT_H
#define OBSTACLEROBOT_H

/**
 * \file ObstacleRobot.h
 * \brief ObstacleRobot class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include "elikos_roomba/Robot.h"

// names
static const std::string OBSTACLEROBOT_TYPE = "obstacle";
// trajectory
static const double CIRC_TRAJECTORY_DIAMETER = 10.0;                                    //[m]
// find angular velocity
static const double CIRC_TRAJECTORY_CIRCUMFERENCE = CIRC_TRAJECTORY_DIAMETER*3.1415;    //[m]
static const double CIRC_TRAJECTORY_DURATION = CIRC_TRAJECTORY_DIAMETER/FORWARD_SPEED;  //[m]/[m/s]=[s]
static const double CIRC_TURN_SPEED = 360.0*DEG_TO_RAD/CIRC_TRAJECTORY_DURATION;        //[deg]*[rad/deg]/[s]=[rad/s]

/** \class ObstacleRobot
 * \brief class which implements obstacle robot behaviour.
 */
class ObstacleRobot : public Robot
{
    public:
        /**
         * \brief ObstacleRobot constructor.
         *
         * \param n : node handle.
         * \param r_id : id of robot.
         * \param initial_pos : initial XYZ position.
         * \param initial_yaw : initial heading.
         * \param height : height of robot (ATTENTION: has to have a corresponding model!).
         */
        ObstacleRobot(ros::NodeHandle& n, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string height);
        
        /**
         * \brief ObstacleRobot destructor.
         */
        ~ObstacleRobot();

        /**
         * \brief Check if current robot is colliding with another robot and react accordingly.
         * --> no collision for obstacle robot right now
         */
        void checkRobotCollision(tf::Vector3 pos) {}

        /**
         * \brief Check if quad is touching topswitch and react accordingly.
         * --> no top interaction obstacle robot
         */
        void checkTopInteraction(tf::Vector3 pos, double diameter) {}

        /**
         * \brief Update robot state; called every spinOnce().
         */
        void update();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();

        /**
         * \brief ROS spin once, called on every loop.
         */
        void spinOnce();

    protected:
        /**
         * \enum ObstacleRobotState
         * \brief obstacle robot state
         */
        enum ObstacleRobotState {
            INACTIVE, /**< inactive */
            CIRCULAR /**< active (moving) */
        };

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update obstacle robot cmd_vel message based on current state.
         */
        void updateState();

        /*===========================
         * Global state
         *===========================*/
        /**
         * \brief Activate global robot state.
         */
        void activateRobot();

        /**
         * \brief Deactivate global robot state.
         */
        void deactivateRobot();

        /*===========================
         * Obstacle robot state changes
         *===========================*/
        /**
         * \brief Change robot state to new given state.
         *
         * \param newRobotState : new state.
         */
        void changeRobotStateTo(ObstacleRobotState newRobotState);

        /**
         * \brief Compare current robot state to given state.
         *
         * \param cmpRobotState : robot state to compare to.
         *
         * \return comparison result.
         */
        bool isRobotState(ObstacleRobotState cmpRobotState);
    
    private:
        ObstacleRobotState robotState_; /**< current obstacle robot state */
};

#endif  // ELIKOS_ROOMBA_OBSTACLEROBOT_H