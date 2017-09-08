#ifndef ROBOT_VIZ_H
#define ROBOT_VIZ_H

#include "elikos_roomba/robot.h"        // use topic names, service names, and other values
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// names (topics and services)
static const std::string TF_NAME_BASE = "elikos_arena_origin";        // origin
static const std::string ROBOTPOSE_TOPIC_NAME = "pose";               // pose message
static const std::string TF_ROBOT_PREFIX = "robot";                   // robot tf name prefix
// marker/model
static const std::string MESH_RESOURCE_PREFIX = "package://elikos_roomba/models/";
static const std::string MARKER_TOPIC_NAME = "marker";
static const int MARKER_TOPIC_QUEUESIZE = 10;
static const std::string OBSTACLE_ROBOT_MODEL_FILE = "obs_10.dae";
static const std::string GROUND_ROBOT_MODEL_FILE_PREFIX = "robot_";
static const std::string GROUND_ROBOT_MODEL_FILE_EXTENSION = "dae";


class RobotViz
{
    private:
        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher pose_pub_;
        tf::TransformBroadcaster tf_br_;
        ros::Publisher marker_pub_;

        /*===========================
         * Subscribers
         *===========================*/
        /* CmdVel subscriber */
        ros::Subscriber cmdVel_sub_;
        /* Robot state subscriber */
        ros::Subscriber robotState_sub_;
    
    protected:
        ros::NodeHandle& n_;
        double loop_hz_;
        bool is_running_slowly_;

        int r_id_;
        std::string robotType_;
        std::string tf_robot_;

        ros::Time time_last_;
        ros::Time time_now_;
        ros::Duration time_diff_;

        std::string mesh_resource_;
        std::string robotColor_;
        
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
        /* Current marker message */
        visualization_msgs::Marker marker_msg_;
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
         * Update; called every spinOnce()
         */
        void update();

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

        /*
         * Publish marker message
         */
        void publishMarker();

        /*
         * ROS spin once, called on every loop
         */
        void spinOnce();
    
    public:
        /*
         * Constructor
         */
        RobotViz(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw, int r_id, std::string robotType, std::string robotColor);
        ~RobotViz();

        /*
         * Create marker message
         */
        void createMarkerMsg();

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

        /*
         * Generate mesh resource for marker depending on robot type and id
         */
        std::string generateMeshResource();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();
};

#endif  // ELIKOS_ROOMBA_ROBOT_VIZ_H