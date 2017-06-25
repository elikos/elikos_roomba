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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <string>
#include <sstream>
#include <cmath>
//#include <Eigen/Core>

static const double LOOP_RATE = 20.0;
// names (topics and services)
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";                     // subscribe to cmd_vel
static const std::string TF_NAME = "robot";                                 // robot tf name
// number parameters
static const int CMDVEL_TOPIC_QUEUESIZE = 30;
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
         tf::TransformBroadcaster tf_br_;

        /*===========================
         * Subscribers
         *===========================*/
         /* CmdVel subscriber */
        ros::Subscriber cmdVel_sub_;
    
    protected:
        ros::NodeHandle& n_;
        double loop_hz_;
        bool is_running_slowly_;

        ros::Time time_last_;
        ros::Time time_new_;
        ros::Duration time_diff_;

        /*===========================
         * Position info
         *===========================*/
         tf::Vector3 initial_pos_;
         double initial_bearing_;
         
         tf::Vector3 pos_;
         double bearing_;

         tf::Transform tf_;

         double linVel_;
         double angVel_;

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for cmd vel topic
         */
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        /*===========================
         * Messages
         *===========================*/


        /*===========================
         * Update
         *===========================*/
        /*
         * Update robot pose; called every spinOnce()
         */
        void update();

        /*
         * ROS spin once, called on every loop
         */
        //virtual void spinOnce() =0;
        void spinOnce();

        /*
         * Update pose
         */
        void updatePose();

        /*
         * Update tf
         */
        void updateTf();
        
        /*
         * Publish robot tf
         */
        void publishRobotTf();
    
    public:
        /*
         * Constructor
         * botType: std::string with type of robot ("GROUND ROBOT" or "OBSTACLE ROBOT")
         */
        RobotSim(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_bearing);
        ~RobotSim();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        //virtual void spin() =0;
        void spin();

        /*
         * Wrapper for ROS_INFO_STREAM, includes robotType_ string in message
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);
};

#endif  // ELIKOS_ROOMBA_ROBOT_SIM_H