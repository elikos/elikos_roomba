#ifndef ROBOT_H
#define ROBOT_H

/**
 * \file Robot.h
 * \brief Robot class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <sstream>
#include "elikos_roomba/MovingObject.h"

static const double LOOP_RATE = 10.0;
// names (topics and services)
static const std::string ROBOTSTATE_TOPIC_NAME = "state";             // publishes current robot state
static const std::string ACTIVATE_SERVICE_NAME = "activate";          // service, activate robot
static const std::string DEACTIVATE_SERVICE_NAME = "deactivate";      // service, deactivate robot
static const std::string TOGGLEACT_SERVICE_NAME = "toggle_activate";  // service, toggle robot activation
static const std::string BUMPER_SERVICE_NAME = "bumper_trigger";      // exposed as feature for robot viz and such
// convention
static const double DEG_TO_RAD = 3.1415/180.0;          //[rad/deg]
static const double ROTATE_CCW = 1.0;                   // counterclockwise (positive angular.z)
static const double ROTATE_CW = -1.0;                   // clockwise (negative angular.z)
// speeds
static const double FORWARD_SPEED = 0.33;               //[m/s]
// physical dimensions
static const double DIAMETER = 0.3485;                  //[m] (according to specs)
static const double HEIGHT = 0.1;                       //[m] (according to specs)
static const double BUMPER_ANGLE = 180.0*DEG_TO_RAD;    //[deg] total angle interval (symmetrical) for bumper

/** \class Robot
 * \brief abstract class which implements basic robot behaviour.
 */
class Robot : public MovingObject
{
    public:
        /**
         * \brief Robot constructor.
         *
         * \param n : node handle.
         * \param botType : type of robot ("ground" or "obstacle").
         * \param r_id : robot ID.
         * \param initial_pos : initial XYZ position.
         * \param initial_yaw : initial heading.
         * \param model_option : characteristic of object (used to get the marker model).
         */
        Robot(ros::NodeHandle& n, std::string botType, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string model_option);
        
        /**
         * \brief Robot destructor.
         */
        ~Robot();

        /**
         * \brief Accessor for current position.
         *
         * \return position.
         */
        /*
         * Check if current robot is colliding with another robot and react accordingly
         */
        virtual void checkRobotCollision(tf::Vector3 pos) =0;

        /**
         * \brief Check if quad is touching topswitch and react accordingly.
         *
         * \param pos : XYZ position of quad.
         * \param diameter : interaction diameter for quad.
         */
        virtual void checkTopInteraction(tf::Vector3 pos, double diameter) =0;

        /**
         * \brief Update robot; called every spinOnce().
         */
        virtual void update();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        virtual void spin() =0;

        /**
         * \brief Robot type accessor.
         *
         * \return robot type.
         */
        std::string getRobotType() const;
    
    protected:
        std_msgs::String robotState_msg_; /**< current robot state message */

        std::string robotType_; /**< robot type */
        int r_id_; /**< robot id */

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update ground robot message based on current state.
         */
        virtual void updateState() =0;

        /**
         * \brief ROS spin once, called on every loop.
         */
        virtual void spinOnce() =0;

        /**
         * \brief Publish a specific CmdVel message (for testing purposes).
         *
         * \param cmdVel_msg : message.
         */
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);

        /**
         * \brief Publish current robot state message.
         */
        void publishRobotState();

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for robot activation service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for robot deactivation service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for robot toggle activate service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*===========================
         * Global state
         *===========================*/
        /**
         * \brief Activate global robot state.
         */
        virtual void activateRobot();

        /**
         * \brief Deactivate global robot state.
         */
        virtual void deactivateRobot();

        /*===========================
         * Other utilities
         *===========================*/
        /**
         * \brief Get CmdVel message (Twist).
         *
         * \param lin_x : linear (x) velocity.
         * \param ang_z : angular (z) velocity.
         *
         * \return twist message.
         */
        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);

        /**
         * \brief Wrapper for ROS_INFO_STREAM, includes robotType_ string and robot ID in message.
         *
         * \param message : message to include.
         */
        void ROS_INFO_STREAM_ROBOT(std::string message);

    private:
        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher robotState_pub_; /**< robot state publisher */

        /*===========================
         * Services
         *===========================*/
        ros::ServiceServer activate_srv_; /**< robot activation service */
        ros::ServiceServer deactivate_srv_; /**< robot deactivation service */
        ros::ServiceServer toglActivate_srv_; /**< robot toggle activate service */

        /**
         * \brief Get namespace.
         *
         * \param robotType : type of robot ("ground" or "obstacle").
         * \param robotId : id of robot.
         *
         * \return robot namespace.
         */
        std::string getRobotNamespace(std::string robotType, int robotId);
};

#endif  // ELIKOS_ROOMBA_ROBOT_H