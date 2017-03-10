#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

#include <ros/ros.h>
#include <ros/timer.h>
#include "elikos_roomba/robot.h"
#include "ca_msgs/Bumper.h"

static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";
static const std::string BUMPER_TOPIC_NAME = "bumper";
static const std::string GROUNDROBOT_TYPE = "GROUND ROBOT";
static const float FORWARD_SPEED = 0.33f; //[m/s]
static const float FORWARD_NOISE = 1.0f;
static const float TURN_SPEED = 90.0f * (3.1415f/180.0f) ; //[deg/s]*[rad/deg]=[rad/s]

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
        ros::Timer twentySec_tim_;
        ros::Timer fiveSec_tim_;
        void twentySecCallback(const ros::TimerEvent& event);
        void fiveSecCallback(const ros::TimerEvent& event);
        ros::Timer bumperTurn_tim_;
        ros::Timer topSwitchTurn_tim_;
        ros::Timer timeoutTurn_tim_;
        void bumperTurnTimCallback(const ros::TimerEvent& event);
        void topSwitchTurnTimCallback(const ros::TimerEvent& event);
        void timeoutTurnTimCallback(const ros::TimerEvent& event);

        // state
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
        void updateCmbVel();
        void updateState();

        void changeRobotStateTo(GroundRobotState newRobotState);
        bool isRobotState(GroundRobotState cmpRobotState);
    
    public:
        GroundRobot(ros::NodeHandle& n);
        ~GroundRobot();
        void spin();
        void spinOnce();
};

#endif  // ELIKOS_ROOMBA_GROUNDROBOT_H reset