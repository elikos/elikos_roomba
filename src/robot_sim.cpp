#include "elikos_roomba/robot_sim.h"

RobotSim::RobotSim(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw)
    : n_(n)
{
    // setup subscribers
    cmdVel_sub_ = n.subscribe(CMDVEL_TOPIC_NAME, 10, &RobotSim::cmdVelCallback, this);
    robotState_sub_ = n.subscribe(ROBOTSTATE_TOPIC_NAME, 10, &RobotSim::robotStateCallback, this);

    // setup publishers
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(ROBOTPOSE_TOPIC_NAME, ROBOTSTATE_TOPIC_QUEUESIZE);

    // initial state
    isActive_ = false;

    // initial pose
    initial_pos_ = initial_pos;
    initial_yaw_ = initial_yaw;
    // current pose
    pos_ = initial_pos_;
    yaw_ = initial_yaw_;

    time_last_ = ros::Time::now(); // could remove this

    // create pose msg
    updatePoseMsg();

    ROS_INFO_STREAM_ROBOT("Initialization done (inactive)");
}

RobotSim::~RobotSim() {
  ROS_INFO_STREAM_ROBOT("Destruct sequence initiated");
  // add other relevant stuff
}

/*===========================
 * Callbacks
 *===========================*/

void RobotSim::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // check if robot is active (and time_last_ reset by robotStateCallback)
    if (isActive_) {
        // time difference
        ros::Time time_now_ = ros::Time::now();
        time_diff_ = time_now_ - time_last_;
        double timeDiffSecs = time_diff_.toSec();

        linVel_ = msg->linear.x;
        angVel_ = msg->angular.z;

        updatePose(timeDiffSecs);
        updatePoseMsg();
        publishPoseMsg();

        time_last_ = time_now_;
    }
}

void RobotSim::robotStateCallback(const std_msgs::String::ConstPtr& msg) {
    bool isActive_now = !((msg->data) == "INACTIVE");

    if (isActive_now && !isActive_) {
        // robot reactivated; reset time
        time_last_ = ros::Time::now();

        ROS_INFO_STREAM_ROBOT("Activated");
    }

    isActive_ = isActive_now;
}

/*===========================
 * Update
 *===========================*/

void RobotSim::updatePose(double timeDiffSecs) {
    // deltas
    double deltaLin = timeDiffSecs*linVel_;
    double deltaAngle = timeDiffSecs*angVel_;

    // components
    double deltaX = cosf(deltaAngle+yaw_)*deltaLin;
    double deltaY = sinf(deltaAngle+yaw_)*deltaLin;

    // add to pose
    pos_ += tf::Vector3(deltaX, deltaY, 0.0);
    yaw_ += deltaAngle;
}

void RobotSim::updatePoseMsg() {
    pose_msg_ = createPoseStampedFromPosYaw(pos_, yaw_);
}

void RobotSim::publishPoseMsg() {
    pose_pub_.publish(pose_msg_);
}

/*===========================
 * Other utilities
 *===========================*/

geometry_msgs::PoseStamped RobotSim::createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = time_last_;
    pose_msg.header.frame_id = TF_NAME_BASE;
    return pose_msg;
}

void RobotSim::ROS_INFO_STREAM_ROBOT(std::string message) {
    ROS_INFO_STREAM("[" << "ROBOT SIM" << "] " << message);
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;

    RobotSim robotsim_(n, tf::Vector3(0.0, 0.0, 0.0), 0.0);
    
    try
    {
        ros::spin();
        //robotsim_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ROBOT SIM] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
