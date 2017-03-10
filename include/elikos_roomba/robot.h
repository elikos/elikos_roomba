#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <string>

static const double LOOP_RATE = 10.0;
// names (topics and services)
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";
static const std::string ROBOTSTATE_TOPIC_NAME = "robot_state";
static const std::string ACTIVATE_SERVICE_NAME = "robot_activate";
static const std::string DEACTIVATE_SERVICE_NAME = "robot_deactivate";
static const std::string TOGGLEACT_SERVICE_NAME = "robot_activate_toggle";
// number parameters
static const int CMDVEL_TOPIC_QUEUESIZE = 50;
static const int ROBOTSTATE_TOPIC_QUEUESIZE = 10;
//static const char* robotTypeText[] = { "GROUND ROBOT", "OBSTACLE ROBOT" };

class Robot
{
    private:
        // publishers
        ros::Publisher cmdVel_pub_;
        ros::Publisher robotState_pub_;

        // services
        ros::ServiceServer activate_srv_;
        ros::ServiceServer deactivate_srv_;
        ros::ServiceServer toglActivate_srv_;
    
    protected:
        ros::NodeHandle& n_;
        std::string robotType_;
        double loop_hz_;
        bool is_running_slowly_;

        // current cmd_vel message
        geometry_msgs::Twist cmdVel_msg_;
        std_msgs::String robotState_msg_;

        // update
        void update();
        void publishCmdVel();
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);
        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);
        void publishRobotState();

        // callbacks
        bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        bool toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        // global state changes
        bool isActive_;
        virtual void activateRobot();
        virtual void deactivateRobot();
    
    public:
        Robot(ros::NodeHandle& n, std::string botType);
        ~Robot();

        // ros spin stuff
        virtual void spin() =0;
        virtual void spinOnce() =0;

        // wrapper for ROS_INFO_STREAM, includes robotType_ string in message
        void ROS_INFO_STREAM_ROBOT(std::string message);
};

#endif  // ELIKOS_ROOMBA_ROBOT_H