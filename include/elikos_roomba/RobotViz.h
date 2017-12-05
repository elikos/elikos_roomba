#ifndef ROBOT_VIZ_H
#define ROBOT_VIZ_H

/**
 * \file RobotViz.h
 * \brief RobotViz class declaration
 * \author christophebedard
 */

#include "elikos_roomba/Robot.h"        // use topic names, service names, and other values
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


// names (topics and services)
static const std::string TF_ROBOT_PREFIX = "robot";                   // robot tf name prefix
// marker/model
static const std::string OBSTACLE_ROBOT_MODEL_FILE = "obs_10.dae";


/** \class RobotViz
 * \brief class which computes and publishes the position of something given a cmd_vel message it subscribes to.
 */
class RobotViz
{
    public:
        /**
         * \brief RobotViz constructor.
         *
         * \param n : node handle.
         * \param initial_pos : initial XYZ position.
         * \param initial_yaw : initial heading.
         * \param r_id : robot ID.
         * \param robotType : type of robot ("ground" or "obstacle").
         * \param robotColor : color of robot ("green" or "red").
         */
        RobotViz(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw, int r_id, std::string robotType, std::string robotColor);
        
        /**
         * \brief RobotViz destructor.
         */
        ~RobotViz();

        /**
         * \brief Create marker message.
         */
        void createMarkerMsg();

        /**
         * \brief Wrapper for ROS_INFO_STREAM, includes robotType_ string and robot ID in message.
         *
         * \param message : message to include.
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);

        /**
         * \brief Create PoseStamped message from position vector and yaw.
         *
         * \param pos : XYZ position.
         * \param yaw : heading (rad).
         *
         * \return PoseStamped message.
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);

        /**
         * \brief Create tf from position vector and yaw.
         *
         * \param pos : XYZ position.
         * \param yaw : heading.
         *
         * \return transform.
         */
        tf::Transform createTfFromPosYaw(tf::Vector3 pos, double yaw);

        /**
         * \brief Generate mesh resource for marker depending on robot type and id.
         *
         * \return mesh resource.
         */
        std::string generateMeshResource();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();
    
    protected:
        ros::NodeHandle& n_; /**< node handle */

        int r_id_; /**< robot id */
        std::string robotType_; /**< robot type */
        std::string robotColor_; /**<  robot color*/
        std::string tf_robot_; /**< tf corresponding to robot */

        ros::Time time_last_; /**< time of last position update */

        std::string mesh_resource_; /**< model name */
        
        /*===========================
         * Robot state
         *===========================*/
        bool isActive_; /**< current active state */

        /*===========================
         * Position info
         *===========================*/
         tf::Vector3 initial_pos_; /**< initial position */
         double initial_yaw_; /**< initial heading */
         
         tf::Vector3 pos_; /**< current position */
         double yaw_; /**< current heading */

        /*===========================
         * Messages
         *===========================*/
        geometry_msgs::PoseStamped pose_msg_; /**< current pose message */
        visualization_msgs::Marker marker_msg_; /**< current marker message */
        tf::Transform tf_; /**< current tf */

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for cmd vel topic.
         *
         * \param msg : constptr to new message.
         */
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        /**
         * \brief Callback class method for robot state topic.
         *
         * \param msg : constptr to new message.
         */
        void robotStateCallback(const std_msgs::String::ConstPtr& msg);

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update; called every spinOnce().
         */
        void update();

        /**
         * \brief Update pose.
         *
         * \param timeDiffSecs : time since last update.
         * \param linVel : linear velocity.
         * \param angVel : angular velocity.
         */
        void updatePose(double timeDiffSecs, double linVel, double angVel);

        /**
         * \brief Update pose messages (PoseStamped and tf).
         */
        void updatePoseMsgs();
        
        /**
         * \brief Publish pose messages (PoseStamped and tf).
         */
        void publishPoseMsgs();

        /**
         * \brief Publish marker message.
         */
        void publishMarker();

        /**
         * \brief ROS spin once, called on every loop.
         */
        void spinOnce();

    private:
        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher pose_pub_; /**< current pose publisher */
        tf::TransformBroadcaster tf_br_; /**< tf boradcaster */
        ros::Publisher marker_pub_; /**< marker publisher */

        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber cmdVel_sub_; /**< CmdVel subscriber */
        ros::Subscriber robotState_sub_; /**< robot state subscriber */
};

#endif  // ELIKOS_ROOMBA_ROBOT_VIZ_H