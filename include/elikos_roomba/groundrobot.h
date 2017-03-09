#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

#include <ros/ros.h>
#include <ros/timer.h>
#include "elikos_roomba/robot.h"
#include "ca_msgs/Bumper.h"

static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";
static const std::string BUMPER_TOPIC_NAME = "bumper";

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

        // state

        /*GroundRobotState grobotState_;
        enum GroundRobotState {
            FORWARD,
            TURN_BUMPER,
            TURN_TOPSWITCH,
            TURN_TIMEOUT
        };*/

    protected:
        void update();
        void updateCmbVel();
    
    public:
        GroundRobot(ros::NodeHandle& n);
        ~GroundRobot();
        void spin();
        void spinOnce();
};

#endif  // ELIKOS_ROOMBA_GROUNDROBOT_H reset