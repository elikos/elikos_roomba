#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H

/**
 * \file MovingObject.h
 * \brief MovingObject class declaration
 * \author christophebedard
 */

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// names (topics and services)
static const std::string TF_NAME_BASE = "elikos_arena_origin";        // origin
static const std::string ROBOTPOSE_TOPIC_NAME = "pose";               // pose message
static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";               // publishes cmd_vel
static const std::string RESET_SERVICE_NAME = "reset";                // to reset robot
// marker/model
static const std::string MESH_RESOURCE_PREFIX = "package://elikos_roomba/models/";
static const std::string MARKER_TOPIC_NAME = "marker";
static const std::string OBSTACLE_ROBOT_MODEL_FILE_PREFIX = "obs_";
static const std::string GROUND_ROBOT_MODEL_FILE_PREFIX = "robot_";
static const std::string ROBOT_MODEL_FILE_EXTENSION = ".dae";

/** \class MovingObject
 * \brief class which computes and publishes the position of a derived object.
 *
 * Given a cmd_vel message.
 */
class MovingObject
{
    public:
        /**
         * \brief MovingObject constructor.
         *
         * \param n : node handle.
         * \param nspace : namespace of object (used as a unique ID).
         * \param initial_pos : initial XYZ position.
         * \param initial_yaw : initial heading.
         * \param model_option : characteristic of object (used to get the marker model).
         */
        MovingObject(ros::NodeHandle& n, std::string nspace, tf::Vector3 initial_pos, double initial_yaw, std::string model_option);
        
        /**
         * \brief MovingObject destructor.
         */
        ~MovingObject();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        virtual void spin() =0;

        /**
         * \brief Accessor for current position.
         *
         * \return position.
         */
        tf::Vector3 getPosition() const;
        
        /**
         * \brief Accessor for current heading.
         *
         * \return heading.
         */
        double getYaw() const;

        /**
         * \brief Accessor for namespace (used as unique ID).
         *
         * \return namespace.
         */
        std::string getNamespace() const;

        /**
         * \brief Accessor for isActive_.
         *
         * \return isActive_.
         */
        bool isActive() const;

        /**
         * \brief Operator== implementation for comparing objects (using namespace).
         *
         * \return result.
         */
        bool operator==(const MovingObject& rhs) const;
    
    protected:
        ros::NodeHandle& n_; /**< node handle */

        std::string ns_; /**< namespace to be used for topics/services (also used as a "unique" identifier) */

        ros::Time time_last_; /**< time of last position update */

        /*===========================
         * State
         *===========================*/
        bool isActive_; /**< current active state */
        bool wasActive_; /**< last state */
        bool isReset_; /**< current reset state */

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
        geometry_msgs::Twist cmdVel_msg_; /**< current CmdVel message (Twist) */
        geometry_msgs::PoseStamped pose_msg_; /**< current pose message */
        visualization_msgs::Marker marker_msg_; /**< current marker message */
        tf::Transform tf_; /**< current pose tf */

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update; called every spinOnce().
         */
        virtual void update();

        /**
         * \brief Update position from cmd_vel.
         */
        void updatePosition();

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
         * \brief Publish the current CmdVel message.
         */
        void publishCmdVel();

        /**
         * \brief Publish a specific CmdVel message (for testing purposes).
         *
         * \param cmdVel_msg : message.
         */
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);

        /**
         * \brief Reset object to initial position and yaw.
         */
        void reset();

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for reset service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    private:
        std::string model_option_; /**< model-specific option (such as colour (groundrobot) or height (obstacle robot)) */
        std::string mesh_resource_; /**< model name (generated) */

        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher cmdVel_pub_; /**< cmd_vel publisher */
        ros::Publisher pose_pub_; /**< pose publisher */
        ros::Publisher marker_pub_; /**< marker publisher */
        tf::TransformBroadcaster tf_br_; /**< pose tf publisher */

        /*===========================
         * Services
         *===========================*/
        ros::ServiceServer reset_srv_; /**< reset service */

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Create marker message (message attributes that do not change).
         */
        void createMarkerMsg();

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
};

#endif  // ELIKOS_ROOMBA_MOVINGOBJECT_H