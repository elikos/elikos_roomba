#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// names (topics and services)
static const std::string TF_NAME_BASE = "elikos_arena_origin";        // origin
static const std::string ROBOTPOSE_TOPIC_NAME = "pose";               // pose message
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";               // publishes cmd_vel
// number parameters
static const int CMDVEL_TOPIC_QUEUESIZE = 30;
static const int POSE_TOPIC_QUEUESIZE = 30;
// marker/model
static const std::string MESH_RESOURCE_PREFIX = "package://elikos_roomba/models/";
static const std::string MARKER_TOPIC_NAME = "marker";
static const int MARKER_TOPIC_QUEUESIZE = 10;
static const std::string OBSTACLE_ROBOT_MODEL_FILE_PREFIX = "obs_";
static const std::string GROUND_ROBOT_MODEL_FILE_PREFIX = "robot_";
static const std::string ROBOT_MODEL_FILE_EXTENSION = ".dae";


class MovingObject
{
    private:
        /*===========================
         * Publishers
         *===========================*/
        /* CmdVel publisher */
        ros::Publisher cmdVel_pub_;

        ros::Publisher pose_pub_;
        tf::TransformBroadcaster tf_br_;
        ros::Publisher marker_pub_;

        /*===========================
         * Utilities
         *===========================*/
        /*
         * Create marker message
         */
        void createMarkerMsg();

        /*
         * Create PoseStamped message from position vector and yaw
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);

        /*
         * Create tf from position vector and yaw
         */
        tf::Transform createTfFromPosYaw(tf::Vector3 pos, double yaw);

        /*
         * Generate mesh resource for marker depending on robot type and id
         */
        std::string generateMeshResource();
    
    protected:
        ros::NodeHandle& n_;

        std::string ns_;

        ros::Time time_last_;
        ros::Time time_now_;
        ros::Duration time_diff_;

        std::string mesh_resource_;
        std::string model_option_;

        /*===========================
         * State
         *===========================*/
        /* Global state (active or not active) */
        bool isActive_;
        bool wasActive_;

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
        /* Current CmdVel message (Twist) */
        geometry_msgs::Twist cmdVel_msg_;

        /* Current pose message */
        geometry_msgs::PoseStamped pose_msg_;
        /* Current marker message */
        visualization_msgs::Marker marker_msg_;
        /* Current tf */
        tf::Transform tf_;

        /*===========================
         * Update
         *===========================*/
        /*
         * Update; called every spinOnce()
         */
        virtual void update();

        /*
         * Update position from cmd_vel
         */
        void updatePosition();

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
         * Publish the current CmdVel message
         */
        void publishCmdVel();

        /*
         * Publish a specific CmdVel message (for testing purposes)
         */
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg); //-----------------------------------
    
    public:
        /*
         * Constructor
         * ns: std::string with namespace to add to topic/srv
         */
        MovingObject(ros::NodeHandle& n, std::string nspace, tf::Vector3 initial_pos, double initial_yaw, std::string model_option);
        ~MovingObject();

        /*
         * Get CmdVel message (Twist) from linear (x) velocity and angular (z) velocity
         */
        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        virtual void spin() =0;

};

#endif  // ELIKOS_ROOMBA_MOVINGOBJECT_H