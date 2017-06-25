#include "elikos_roomba/robot_sim.h"

RobotSim::RobotSim(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_bearing)
    : n_(n),
      is_running_slowly_(false)
{
    loop_hz_ = LOOP_RATE;

    // setup publishers
    

    // setup subscribers
    cmdVel_sub_ = n.subscribe(CMDVEL_TOPIC_NAME, 10, &RobotSim::cmdVelCallback, this);

    // setup services


    // initial state
    initial_pos_ = initial_pos;
    initial_bearing_ = initial_bearing;

    pos_ = initial_pos_;
    bearing_ = initial_bearing_;

    tf_.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, bearing_);
    tf_.setRotation(q);

    time_last_ = ros::Time::now();
}

RobotSim::~RobotSim() {
  ROS_INFO_STREAM_ROBOT("Robot base destruct robot sequence initiated");
  // add other relevant stuff
}

/*===========================
 * Other utilities
 *===========================*/

void RobotSim::ROS_INFO_STREAM_ROBOT(std::string message) {
    ROS_INFO_STREAM("[" << "robot_sim" << "] " << message);
}

/*===========================
 * Callbacks
 *===========================*/

void RobotSim::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    linVel_ = msg->linear.x;
    angVel_ = msg->angular.z;
}

/*===========================
 * Update
 *===========================*/
void RobotSim::updatePose() {
    // time
    ros::Time time_now_ = ros::Time::now();
    time_diff_ = time_now_ - time_last_;
    double timeDiffSecs = time_diff_.toSec();
    //TODO use cmd vel message time difference (and stop updating if no more messages)

    // deltas
    double deltaLin = timeDiffSecs*linVel_;
    double deltaAngle = timeDiffSecs*angVel_;

    // components
    double deltaX = cosf(deltaAngle+bearing_)*deltaLin;
    double deltaY = sinf(deltaAngle+bearing_)*deltaLin;

    // add to pose
    pos_ += tf::Vector3(deltaX, deltaY, 0.0);
    bearing_ += deltaAngle;

    time_last_ = time_now_;
}

void RobotSim::updateTf() {
    tf_.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, bearing_);
    tf_.setRotation(q);
}

void RobotSim::publishRobotTf() {
    tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "world", "robot_pose"));
}

void RobotSim::update() {
    //ROS_INFO_STREAM_ROBOT("update");

    updatePose();
    updateTf();
    publishRobotTf();
}

void RobotSim::spinOnce()
{
  update();
  ros::spinOnce();
}

void RobotSim::spin()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[ROBOT SIM] Loop running slowly.");
    }
  }
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;

    RobotSim robotsim_(n, tf::Vector3(0.0, 0.0, 0.0), 0.0);

    try
    {
        robotsim_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ROBOT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
