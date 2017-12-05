/**
 * \file MovingObject.cpp
 * \brief MovingObject class implementation
 * \author christophebedard
 */

#include "elikos_roomba/MovingObject.h"

MovingObject::MovingObject(ros::NodeHandle& n, std::string nspace, tf::Vector3 initial_pos, double initial_yaw, std::string model_option)
    : n_(n),
      ns_(nspace),
      model_option_(model_option)
{
    // setup publishers
    cmdVel_pub_ = n.advertise<geometry_msgs::Twist>(ns_ + "/" + CMDVEL_TOPIC_NAME, 30);
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(ns_ + "/" + ROBOTPOSE_TOPIC_NAME, 30);
    marker_pub_ = n.advertise<visualization_msgs::Marker>(ns_ + "/" + MARKER_TOPIC_NAME, 10);

    // setup services
    reset_srv_ = n.advertiseService(ns_ + "/" + RESET_SERVICE_NAME, &MovingObject::resetCallback, this);

    // initial state
    isActive_ = false;
    wasActive_ = false;
    isReset_ = false;

    // initial pose
    initial_pos_ = initial_pos;
    initial_yaw_ = initial_yaw;
    // current pose
    pos_ = initial_pos_;
    yaw_ = initial_yaw_;

    time_last_ = ros::Time::now(); // could remove this

    // create pose msgs
    updatePoseMsgs();

    // generate mesh resource for marker
    mesh_resource_ = generateMeshResource();
    // create marker message
    createMarkerMsg();
}

MovingObject::~MovingObject() {
}

/*===========================
 * Other utilities
 *===========================*/

std::string MovingObject::generateMeshResource() {
    std::string frame_res;

    if (ns_.find("obstacle") != std::string::npos) {
        frame_res = OBSTACLE_ROBOT_MODEL_FILE_PREFIX + model_option_ + ROBOT_MODEL_FILE_EXTENSION;
    } else if (ns_.find("ground") != std::string::npos) {
        frame_res = GROUND_ROBOT_MODEL_FILE_PREFIX + model_option_ + ROBOT_MODEL_FILE_EXTENSION;
    }
    
    return frame_res;
}

void MovingObject::createMarkerMsg() {
    marker_msg_.header.frame_id = "/" + ns_;
    //marker_msg.ns = "myns";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg_.action = visualization_msgs::Marker::ADD;
    marker_msg_.mesh_resource = MESH_RESOURCE_PREFIX + mesh_resource_;
    marker_msg_.pose.orientation.w = 1.0;
    marker_msg_.scale.x = 1.0;
    marker_msg_.scale.y = 1.0;
    marker_msg_.scale.z = 1.0;
    marker_msg_.mesh_use_embedded_materials = true;
    marker_msg_.lifetime = ros::Duration();
}

geometry_msgs::PoseStamped MovingObject::createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = time_last_;
    pose_msg.header.frame_id = TF_NAME_BASE;
    return pose_msg;
}

tf::Transform MovingObject::createTfFromPosYaw(tf::Vector3 pos, double yaw) {
    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin(tf::Vector3(pos.x(), pos.y(), 0.0));
    q.setRPY(0, 0, yaw);
    tf.setRotation(q);
    return tf;
}

/*===========================
 * Callbacks
 *===========================*/

bool MovingObject::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    reset();
    return true;
}

/*===========================
 * Update
 *===========================*/

void MovingObject::reset() {
    isReset_ = true;
    pos_ = initial_pos_;
    yaw_ = initial_yaw_;
}

 void MovingObject::publishCmdVel() {
    cmdVel_pub_.publish(cmdVel_msg_);
}

void MovingObject::publishCmdVel(geometry_msgs::Twist msg_) {
    cmdVel_pub_.publish(msg_);
}

void MovingObject::publishPoseMsgs() {
    pose_pub_.publish(pose_msg_);
    tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), TF_NAME_BASE, ns_));
}

void MovingObject::publishMarker() {
    marker_msg_.header.stamp = ros::Time::now();
    marker_pub_.publish(marker_msg_);
}

void MovingObject::updatePose(double timeDiffSecs, double linVel, double angVel) {
    // deltas
    double deltaLin = timeDiffSecs*linVel;
    double deltaAngle = timeDiffSecs*angVel;

    // components
    double deltaX = cosf(deltaAngle+yaw_)*deltaLin;
    double deltaY = sinf(deltaAngle+yaw_)*deltaLin;

    // add to pose
    pos_ += tf::Vector3(deltaX, deltaY, 0.0);
    yaw_ += deltaAngle;
}

void MovingObject::updatePoseMsgs() {
    pose_msg_ = createPoseStampedFromPosYaw(pos_, yaw_);
    tf_ = createTfFromPosYaw(pos_, yaw_);
}

void MovingObject::updatePosition() {
    // check if robot was just activated
    if (isActive_ && !wasActive_) {
        // robot reactivated; reset time
        time_last_ = ros::Time::now();
    }

    // check if robot was reset
    if (isReset_) {
        isReset_ = false;
        // update pose msgs following reset
        updatePoseMsgs();
    }
    
    // check if robot is active
    if (isActive_) {
        // time difference
        ros::Time time_now = ros::Time::now();
        ros::Duration time_diff = time_now - time_last_;
        double timeDiffSecs = time_diff.toSec();

        double linVel = cmdVel_msg_.linear.x;
        double angVel = cmdVel_msg_.angular.z;

        updatePose(timeDiffSecs, linVel, angVel);
        updatePoseMsgs();

        time_last_ = time_now;
    }

    wasActive_ = isActive_;
}

void MovingObject::update() {
    updatePosition();
    publishPoseMsgs();
    publishMarker();
}

/*===========================
 * Accessors
 *===========================*/

 tf::Vector3 MovingObject::getPosition() const {
     return pos_;
 }

 double MovingObject::getYaw() const {
     return yaw_;
 }

 std::string MovingObject::getNamespace() const {
     return ns_;
 }