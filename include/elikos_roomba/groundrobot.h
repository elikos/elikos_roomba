#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

#include <ros/ros.h>
#include <ros/timer.h>
#include "elikos_roomba/robot.h"
#include "ca_msgs/Bumper.h"

// names
static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";          // called by topswitch_node
static const std::string BUMPER_TOPIC_NAME = "bumper";                          // published by create_autonomy
static const std::string GROUNDROBOT_TYPE = "GROUND ROBOT";
// speeds
static const float FORWARD_SPEED = 0.33f;                                       //[m/s]
static const float TURN_SPEED = 90.0f*DEG_TO_RAD;                               //[deg/s]*[rad/deg]=[rad/s]
static const float NOISE_TURN_SPEED = 90.0f*DEG_TO_RAD;                         //[deg/s]*[rad/deg]=[rad/s]
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
    private:
        // service
        ros::ServiceServer topSwitch_srv_;
        bool topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        // subscriber
        ros::Subscriber bumper_sub_;
        void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);

        // timers
        ros::Timer timeout_tim_;        // every 20 seconds, turn around
        ros::Timer noise_tim_;          // every 5 seconds, start to add angle noise to forward path
        ros::Timer noiseTurn_tim_;      // for noise turn
        ros::Timer bumperTurn_tim_;     // for turn after bumper collision
        ros::Timer topSwitchTurn_tim_;  // for turn after top switch activation
        ros::Timer timeoutTurn_tim_;    // for turn after timeout
        // timer callbacks
        void timeoutCallback(const ros::TimerEvent& event);
        void noiseCallback(const ros::TimerEvent& event);
        void noiseTurnCallback(const ros::TimerEvent& event);
        void bumperTurnTimCallback(const ros::TimerEvent& event);
        void topSwitchTurnTimCallback(const ros::TimerEvent& event);
        void timeoutTurnTimCallback(const ros::TimerEvent& event);

        // noise generation
        double getRandomNoiseAngle();
        double getTurnDurationFromAngleAndSpeed(double angl, double speed);

        // ground robot states
        enum GroundRobotState {
            INACTIVE,
            FORWARD,
            TURN_BUMPER,
            TURN_TOPSWITCH,
            TURN_TIMEOUT
        };
        GroundRobotState robotState_;

        // actions
        void startBumperTurn();
        void startTopSwitchTurn();
        void startTimeoutTurn();

        float forward_noise_;

    protected:
        void update();
        void updateState();

        // ground robot state changes
        void changeRobotStateTo(GroundRobotState newRobotState);
        bool isRobotState(GroundRobotState cmpRobotState);

        // global state changes
        void activateRobot();
        void deactivateRobot();

        // timer reset
        void timerRestart(ros::Timer tim, double dur);
    
    public:
        GroundRobot(ros::NodeHandle& n);
        ~GroundRobot();
        void spin();
        void spinOnce();
};

#endif  // ELIKOS_ROOMBA_GROUNDROBOT_H reset