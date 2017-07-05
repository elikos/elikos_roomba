#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <string>
#include <sstream>
#include <cmath>
//#include <Eigen/Core>

static const double LOOP_RATE = 20.0;
// names (topics and services)
static const std::string TF_NAME_BASE = "elikos_arena_origin";              // origin
static const std::string ROBOTPOSE_TOPIC_NAME = "pose";                     // pose message
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";                     // subscribe to cmd_vel
static const std::string ROBOTSTATE_TOPIC_NAME = "robot_state";             // subscribe to robot_state
static const std::string TF_NAME = "robot";                                 // robot tf name
static const std::string ACTIVATE_SERVICE_NAME = "robot_activate";          // service, activate robot
static const std::string DEACTIVATE_SERVICE_NAME = "robot_deactivate";      // service, deactivate robot
static const std::string TOGGLEACT_SERVICE_NAME = "robot_activate_toggle";  // service, toggle robot activation
// number parameters
static const int ROBOTSTATE_TOPIC_QUEUESIZE = 10;
// convention
static const double DEG_TO_RAD = 3.1415/180.0;  //[rad/deg]
static const double ROTATE_CCW = 1.0;           // counterclockwise (positive angular.z)
static const double ROTATE_CW = -1.0;           // clockwise (negative angular.z)


class RobotSim
{
    private:
        /*===========================
         * Publishers
         *===========================*/
         ros::Publisher pose_pub_;

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

         geometry_msgs::PoseStamped pose_msg_;

         double linVel_;
         double angVel_;

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
         * Messages
         *===========================*/


        /*===========================
         * Update
         *===========================*/
        /*
         * Update pose
         */
        void updatePose(double timeDiffSecs);

        /*
         * Update pose message
         */
        void updatePoseMsg();
        
        /*
         * Publish robot pose
         */
        void publishPoseMsg();
    
    public:
        /*
         * Constructor
         * botType: std::string with type of robot ("GROUND ROBOT" or "OBSTACLE ROBOT")
         */
        RobotSim(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw, int r_id);
        ~RobotSim();

        /*
         * Wrapper for ROS_INFO_STREAM, includes robotType_ string in message
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);

        /*
         * Create PoseStamped message from position vector and yaw
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);
};

#endif  // ELIKOS_ROOMBA_ROBOT_SIM_H