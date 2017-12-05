#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

#include <ros/ros.h>
#include <ros/timer.h>
#include "elikos_roomba/robot.h"
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


class GroundRobot : public Robot
{
    public:
        GroundRobot(ros::NodeHandle& n, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string color);
        ~GroundRobot();

        /*
         * Check if current robot is colliding with another robot and react accordingly
         */
        void checkRobotCollision(tf::Vector3 pos);

        /*
         * Check if quad is touching topswitch and react accordingly
         */
        void checkTopInteraction(tf::Vector3 pos, double diameter);

        /*
         * Update robot state; called every spinOnce()
         */
        void update();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();

        /*
         * ROS spin once, called on every loop
         */
        void spinOnce();

    protected:
        /*===========================
         * Ground robot state
         *===========================*/
        enum GroundRobotState {
            INACTIVE,
            FORWARD,
            TURN_BUMPER,
            TURN_TOPSWITCH,
            TURN_TIMEOUT
        };

        /*===========================
         * Update
         *===========================*/
        /*
         * Update ground robot message based on current state
         */
        void updateState();

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
         * Ground robot state changes
         *===========================*/
        /*
         * Change robot state to new given state
         */
        void changeRobotStateTo(GroundRobotState newRobotState);

        /*
         * Compare current robot state to given state
         */
        bool isRobotState(GroundRobotState cmpRobotState);
    
    private:
        GroundRobotState robotState_;

        /*===========================
         * Services
         *===========================*/
        /* Top switch service */
        ros::ServiceServer topSwitch_srv_;

        /* Bumper service */
        ros::ServiceServer bumper_srv_;

        /*===========================
         * Subscribers
         *===========================*/
        /* Bumper state subscriber */
        ros::Subscriber bumper_sub_;

        /*===========================
         * Timers
         *===========================*/
        ros::Timer timeout_tim_;        // every 20 seconds, turn around
        ros::Timer noise_tim_;          // every 5 seconds, start to add angle noise to forward path
        ros::Timer noiseTurn_tim_;      // for noise turn
        ros::Timer bumperTurn_tim_;     // for turn after bumper collision
        ros::Timer topSwitchTurn_tim_;  // for turn after top switch activation
        ros::Timer timeoutTurn_tim_;    // for turn after timeout

        /*
         * Restart timer with duration
         */
        void timerRestart(ros::Timer& tim, double dur);

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for top switch activation
         */
        bool topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Callback class method for bumper contact
         */
        void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);

        /*
         * Callback class method for bumper trigger service (used only by sim-related nodes; create_autonomy uses a topic)
         */
        bool bumperTrigCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Method for bumper trigger (used by bumperCallback() and bumperTrigCallback())
         */
        void bumperTrig();

        /*
         * Method for topswitch trigger
         */
        void topswitchTrig();

        /*
         * Callback class method for timeout timer
         */
        void timeoutCallback(const ros::TimerEvent& event);

        /*
         * Callback class method for noise timer timer
         */
        void noiseCallback(const ros::TimerEvent& event);

        /*
         * Callback class method for noise turn timer timer
         */
        void noiseTurnCallback(const ros::TimerEvent& event);

        /*
         * Callback class method for bumper turn end timer
         */
        void bumperTurnTimCallback(const ros::TimerEvent& event);

        /*
         * Callback class method for top switch turn end timer
         */
        void topSwitchTurnTimCallback(const ros::TimerEvent& event);

        /*
         * Callback class method for timeout turn end timer
         */
        void timeoutTurnTimCallback(const ros::TimerEvent& event);

        /*===========================
         * Actions
         *===========================*/
        /*
         * Setup and start turn after bumper contact
         */
        void startBumperTurn();

        /*
         * Setup and start turn after top switch activation
         */
        void startTopSwitchTurn();

        /*
         * Setup and start turn after forward trajectory timeout
         */
        void startTimeoutTurn();

        /*===========================
         * Other utilities
         *===========================*/
        /*
         * Get new random noise angle according to NOISE_ANGLE_MIN, NOISE_ANGLE_MAX, and NOISE_ANGLE_MIN
         */
        double getRandomNoiseAngle();

        /* Angular noise to add to forward trajectory */
        float forward_noise_;

        /*
         * Get turn duration from angular velocity and total angle of turn
         */
        double getTurnDurationFromAngleAndSpeed(double angl, double speed);
};

#endif  // ELIKOS_ROOMBA_GROUNDROBOT_H