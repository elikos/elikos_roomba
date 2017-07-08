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
#include <sstream>

static const double LOOP_RATE = 10.0;
// names (topics and services)
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";                     // publishes cmd_vel
static const std::string ROBOTSTATE_TOPIC_NAME = "robot_state";             // publishes current robot state
static const std::string ACTIVATE_SERVICE_NAME = "robot_activate";          // service, activate robot
static const std::string DEACTIVATE_SERVICE_NAME = "robot_deactivate";      // service, deactivate robot
static const std::string TOGGLEACT_SERVICE_NAME = "robot_activate_toggle";  // service, toggle robot activation
// number parameters
static const int CMDVEL_TOPIC_QUEUESIZE = 30;
static const int ROBOTSTATE_TOPIC_QUEUESIZE = 10;
// convention
static const double DEG_TO_RAD = 3.1415/180.0;  //[rad/deg]
static const double ROTATE_CCW = 1.0;           // counterclockwise (positive angular.z)
static const double ROTATE_CW = -1.0;           // clockwise (negative angular.z)
// speeds
static const float FORWARD_SPEED = 0.33f;       //[m/s]


class Robot
{
    private:
        /*===========================
         * Publishers
         *===========================*/
        /* CmdVel publisher */
        ros::Publisher cmdVel_pub_;
        /* Robot state publisher */
        ros::Publisher robotState_pub_;

        /*===========================
         * Services
         *===========================*/
        /* Robot activation service  */
        ros::ServiceServer activate_srv_;
        /* Robot deactivation service */
        ros::ServiceServer deactivate_srv_;
        /* Robot toggle activate service */
        ros::ServiceServer toglActivate_srv_;
    
    protected:
        ros::NodeHandle& n_;
        double loop_hz_;
        bool is_running_slowly_;

        /*===========================
         * Messages
         *===========================*/
        /* Current CmdVel message (Twist) */
        geometry_msgs::Twist cmdVel_msg_;

        /* Current robot state message (String) */
        std_msgs::String robotState_msg_;

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
        virtual void updateState() =0;

        /*
         * ROS spin once, called on every loop
         */
        virtual void spinOnce() =0;

        /*
         * Publish the current CmdVel message
         */
        void publishCmdVel();

        /*
         * Publish a specific CmdVel message (for testing purposes)
         */
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);

        /*
         * Get CmdVel message (Twist) from linear (x) velocity and angular (z) velocity
         */
        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);

        /*
         * Publish robot state message (String)
         */
        void publishRobotState();

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for robot activation service
         */
        bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Callback class method for robot deactivation service
         */
        bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Callback class method for robot toggle activate service
         */
        bool toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*===========================
         * Global robot state
         *===========================*/
        /* Global robot state (active or not active) */
        bool isActive_;

        /* Robot type */
        std::string robotType_;

        /* Robot id */
        int r_id_;

        /*
         * Activate global robot state
         */
        virtual void activateRobot();

        /*
         * Deactivate global robot state
         */
        virtual void deactivateRobot();
    
    public:
        /*
         * Constructor
         * botType: std::string with type of robot ("GROUND ROBOT" or "OBSTACLE ROBOT")
         */
        Robot(ros::NodeHandle& n, std::string botType, int r_id);
        ~Robot();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        virtual void spin() =0;

        /*
         * Wrapper for ROS_INFO_STREAM, includes robotType_ string and robot ID in message
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);
};

#endif  // ELIKOS_ROOMBA_ROBOT_H