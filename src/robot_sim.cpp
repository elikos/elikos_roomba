#include "elikos_roomba/robot_sim.h"

RobotSim::RobotSim(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_bearing)
    : n_(n)
{
    // setup publishers
    

    // setup subscribers
    cmdVel_sub_ = n.subscribe(CMDVEL_TOPIC_NAME, 10, &RobotSim::cmdVelCallback, this);
    robotState_sub_ = n.subscribe(ROBOTSTATE_TOPIC_NAME, 10, &RobotSim::robotStateCallback, this);

    // initial state
    isActive_ = false;

    initial_pos_ = initial_pos;
    initial_bearing_ = initial_bearing;

    pos_ = initial_pos_;
    bearing_ = initial_bearing_;

    tf_.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, bearing_);
    tf_.setRotation(q);

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
    // time difference
    ros::Time time_now_ = ros::Time::now();
    time_diff_ = time_now_ - time_last_;
    double timeDiffSecs = time_diff_.toSec();

    linVel_ = msg->linear.x;
    angVel_ = msg->angular.z;

    updatePose(timeDiffSecs);
    updateTf();
    publishRobotTf();

    time_last_ = time_now_;
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
    double deltaX = cosf(deltaAngle+bearing_)*deltaLin;
    double deltaY = sinf(deltaAngle+bearing_)*deltaLin;

    // add to pose
    pos_ += tf::Vector3(deltaX, deltaY, 0.0);
    bearing_ += deltaAngle;
}

void RobotSim::updateTf() {
    tf_.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, bearing_);
    tf_.setRotation(q);
}

void RobotSim::publishRobotTf() {
    tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), TF_NAME_BASE, TF_NAME_ROBOT));
}

/*===========================
 * Other utilities
 *===========================*/

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
