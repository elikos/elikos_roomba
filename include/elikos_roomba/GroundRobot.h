#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

/**
 * \file GroundRobot.h
 * \brief GroundRobot class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include <ros/timer.h>
#include "elikos_roomba/Robot.h"
#include "ca_msgs/Bumper.h"

// names
static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";          // called by topswitch_node
static const std::string BUMPER_TOPIC_NAME = "bumper";                          // published by create_autonomy
static const std::string GROUNDROBOT_TYPE = "ground";
// speeds
static const double TURN_SPEED = 90.0*DEG_TO_RAD;                               //[deg/s]*[rad/deg]=[rad/s]
static const double NOISE_TURN_SPEED = 90.0*DEG_TO_RAD;                         //[deg/s]*[rad/deg]=[rad/s]
// angles (direction of rotation is applied in updateState())
static const double BUMPER_TURN_ANGLE = 180.0*DEG_TO_RAD;                       //[deg]*[rad/deg]=[rad]
static const double TOPSWITCH_TURN_ANGLE = 45.0*DEG_TO_RAD;                     //[deg]*[rad/deg]=[rad]
static const double TIMEOUT_TURN_ANGLE = 180.0*DEG_TO_RAD;                      //[deg]*[rad/deg]=[rad]
static const double NOISE_ANGLE_MIN = 0.0*DEG_TO_RAD;                           //[deg]*[rad/deg]=[rad]
static const double NOISE_ANGLE_MAX = 20.0*DEG_TO_RAD;                          //[deg]*[rad/deg]=[rad]
// durations
static const double BUMPER_TURN_DURATION = BUMPER_TURN_ANGLE/TURN_SPEED;        //[rad]/[rad/s]=[s]
static const double TOPSWITCH_TURN_DURATION = TOPSWITCH_TURN_ANGLE/TURN_SPEED;  //[rad]/[rad/s]=[s]
static const double TIMEOUT_TURN_DURATION = TIMEOUT_TURN_ANGLE/TURN_SPEED;      //[rad]/[rad/s]=[s]
static const double TIMEOUT_DURATION = 20.0;                                    //[s] turn every 20 s
static const double NOISE_DURATION = 5.0;                                       //[s] noise every 5 s

/** \class GroundRobot
 * \brief class which implements target robot behaviour.
 */
class GroundRobot : public Robot
{
    public:
        /**
         * \brief GroundRobot constructor.
         *
         * \param n : node handle.
         * \param r_id : id of robot.
         * \param initial_pos : initial XYZ position.
         * \param initial_yaw : initial heading.
         * \param color : color of robot ("green" or "red").
         */
        GroundRobot(ros::NodeHandle& n, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string color);
        
        /**
         * \brief GroundRobot destructor.
         */
        ~GroundRobot();

        /**
         * \brief Check if current robot is colliding with another robot and react accordingly.
         *
         * \param pos : XYZ position of other robot.
         */
        virtual void checkRobotCollision(tf::Vector3 pos);

        /**
         * \brief Check if quad is touching topswitch and react accordingly.
         *
         * \param pos : XYZ position of quad.
         * \param diameter : diameter to consider for interaction.
         */
        virtual void checkTopInteraction(tf::Vector3 pos, double diameter);

        /**
         * \brief Update robot state; called every spinOnce().
         */
        virtual void update();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        virtual void spin();

        /**
         * \brief ROS spin once, called on every loop.
         */
        void spinOnce();

        /*===========================
         * Global state
         *===========================*/
        /**
         * \brief Activate global robot state.
         */
        virtual void activateRobot();

        /**
         * \brief Deactivate global robot state.
         */
        virtual void deactivateRobot();

    protected:
        /**
         * \enum GroundRobotState
         * \brief target robot state
         */
        enum GroundRobotState {
            INACTIVE, /**< inactive */
            FORWARD, /**< moving forward */
            TURN_BUMPER, /**< turning (bumper) */
            TURN_TOPSWITCH, /**< turning (topwitch) */
            TURN_TIMEOUT /**< turning (timeout) */
        };

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update ground robot cmd_vel message based on current state.
         */
        void updateState();

        /*===========================
         * Ground robot state changes
         *===========================*/
        /**
         * \brief Change robot state to new given state.
         *
         * \param newRobotState : new state.
         */
        void changeRobotStateTo(GroundRobotState newRobotState);

        /**
         * \brief Compare current robot state to given state.
         *
         * \param cmpRobotState : robot state to compare to.
         *
         * \return comparison result.
         */
        bool isRobotState(GroundRobotState cmpRobotState);
    
    private:
        GroundRobotState robotState_; /**< current target robot state */
        double forward_noise_; /**< angular noise to add to forward trajectory */

        /*===========================
         * Services
         *===========================*/
        ros::ServiceServer topSwitch_srv_; /**< top switch service */
        ros::ServiceServer bumper_srv_; /**< bumper service */

        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber bumper_sub_; /**< bumper state subscriber */

        /*===========================
         * Timers
         *===========================*/
        ros::Timer timeout_tim_; /**< every 20 seconds, turn around */
        ros::Timer noise_tim_; /**< every 5 seconds, start to add angle noise to forward path */
        ros::Timer noiseTurn_tim_; /**< for noise turn */
        ros::Timer bumperTurn_tim_; /**< for turn after bumper collision */
        ros::Timer topSwitchTurn_tim_; /**< for turn after top switch activation */
        ros::Timer timeoutTurn_tim_; /**< for turn after timeout */

        /**
         * \brief Restart given timer with duration.
         *
         * \param tim : reference to timer.
         * \param dur : duration.
         */
        void timerRestart(ros::Timer& tim, double dur);

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for top switch activation.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for bumper status.
         *
         * \param msg : constptr to message.
         */
        void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);

        /**
         * \brief Callback class method for bumper trigger service.
         * (used only by sim-related nodes; create_autonomy uses a topic).
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool bumperTrigCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for timeout timer.
         *
         * \param event : timer event.
         */
        void timeoutCallback(const ros::TimerEvent& event);

        /**
         * \brief Callback class method for noise timer timer.
         *
         * \param event : timer event.
         */
        void noiseCallback(const ros::TimerEvent& event);

        /**
         * \brief Callback class method for noise turn timer timer.
         *
         * \param event : timer event.
         */
        void noiseTurnCallback(const ros::TimerEvent& event);

        /**
         * \brief Callback class method for bumper turn end timer.
         *
         * \param event : timer event.
         */
        void bumperTurnTimCallback(const ros::TimerEvent& event);

        /**
         * \brief Callback class method for top switch turn end timer.
         *
         * \param event : timer event.
         */
        void topSwitchTurnTimCallback(const ros::TimerEvent& event);

        /**
         * \brief Callback class method for timeout turn end timer.
         *
         * \param event : timer event.
         */
        void timeoutTurnTimCallback(const ros::TimerEvent& event);

        /*===========================
         * Actions
         *===========================*/
        /**
         * \brief Method for bumper trigger (used by bumperCallback() and bumperTrigCallback()).
         */
        void bumperTrig();

        /**
         * \brief Method for topswitch trigger.
         */
        void topswitchTrig();
        
        /**
         * \brief Setup and start turn after bumper contact.
         */
        void startBumperTurn();

        /**
         * \brief Setup and start turn after top switch activation.
         */
        void startTopSwitchTurn();

        /**
         * \brief Setup and start turn after forward trajectory timeout.
         */
        void startTimeoutTurn();

        /*===========================
         * Other utilities
         *===========================*/
        /**
         * \brief Get new random noise angle according to NOISE_ANGLE_MIN, NOISE_ANGLE_MAX, and NOISE_ANGLE_MIN.
         */
        double getRandomNoiseAngle();

        /**
         * \brief Get turn duration.
         *
         * \param angl : angle (rad).
         * \param speed : speed (rad/s).
         *
         * \return turn duration (s).
         */
        double getTurnDurationFromAngleAndSpeed(double angl, double speed);
};

#endif  // ELIKOS_ROOMBA_GROUNDROBOT_H