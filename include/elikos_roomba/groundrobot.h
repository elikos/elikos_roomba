#ifndef GROUNDROBOT_H
#define GROUNDROBOT_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "elikos_roomba/robot.h"

static const std::string TOPSWITCH_SERVICE_NAME = "topswitch_trigger";

class GroundRobot : public Robot
{
    private:
        // service
        ros::ServiceServer topSwitch_srv_;
        bool topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        // timers

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