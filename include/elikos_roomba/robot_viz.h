#ifndef ROBOT_VIZ_H
#define ROBOT_VIZ_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <string>
#include <sstream>
#include <cmath>

static const double LOOP_RATE = 20.0;
// names (topics and services)
static const std::string TF_NAME_BASE = "elikos_arena_origin";        // origin
static const std::string ROBOTPOSE_TOPIC_NAME = "pose";               // pose message
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";               // subscribe to cmd_vel
static const std::string ROBOTSTATE_TOPIC_NAME = "state";             // subscribe to robot_state
static const std::string TF_ROBOT_PREFIX = "robot";                   // robot tf name prefix
static const std::string ACTIVATE_SERVICE_NAME = "activate";          // service, activate robot
static const std::string DEACTIVATE_SERVICE_NAME = "deactivate";      // service, deactivate robot
static const std::string TOGGLEACT_SERVICE_NAME = "toggle_activate";  // service, toggle robot activation
// number parameters
static const int ROBOTSTATE_TOPIC_QUEUESIZE = 10;
// convention
static const double DEG_TO_RAD = 3.1415/180.0;  //[rad/deg]
static const double ROTATE_CCW = 1.0;           // counterclockwise (positive angular.z)
static const double ROTATE_CW = -1.0;           // clockwise (negative angular.z)


class RobotViz
{
    private:
        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher pose_pub_;
        tf::TransformBroadcaster tf_br_;

        /*===========================
         * Subscribers
         *===========================*/
        /* CmdVel subscriber */
        ros::Subscriber cmdVel_sub_;
        /* Robot state subscriber */
        ros::Subscriber robotState_sub_;
    
    protected:
        ros::NodeHandle& n_;

        int r_id_;
        std::string tf_robot_;

        ros::Time time_last_;
        ros::Time time_now_;
        ros::Duration time_diff_;
        
        /*===========================
         * Robot state
         *===========================*/
        bool isActive_;

        /*===========================
         * Position info
         *===========================*/
         tf::Vector3 initial_pos_;
         double initial_yaw_;
         
         tf::Vector3 pos_;
         double yaw_;

        /*===========================
         * Messages
         *===========================*/
        /* Current pose message */
        geometry_msgs::PoseStamped pose_msg_;
        /* Current tf */
        tf::Transform tf_;

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for cmd vel topic
         */
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        /*
         * Callback class method for robot state topic
         */
        void robotStateCallback(const std_msgs::String::ConstPtr& msg);

        /*===========================
         * Update
         *===========================*/
        /*
         * Update pose
         */
        void updatePose(double timeDiffSecs, double linVel, double angVel);

        /*
         * Update pose messages (PoseStamped and tf)
         */
        void updatePoseMsgs();
        
        /*
         * Publish pose messages (PoseStamped and tf)
         */
        void publishPoseMsgs();
    
    public:
        /*
         * Constructor
         * botType: std::string with type of robot ("GROUND ROBOT" or "OBSTACLE ROBOT")
         */
        RobotViz(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw, int r_id);
        ~RobotViz();

        /*
         * Wrapper for ROS_INFO_STREAM, includes robotType_ string and robot ID in message
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);

        /*
         * Create PoseStamped message from position vector and yaw
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);

        /*
         * Create tf from position vector and yaw
         */
        tf::Transform createTfFromPosYaw(tf::Vector3 pos, double yaw);

        /*
         * Concatenate string and int (because other methods weren't working)
         */
        std::string catStringInt(std::string strng, int eent);
};

#endif  // ELIKOS_ROOMBA_ROBOT_VIZ_H