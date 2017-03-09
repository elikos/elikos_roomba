#include "elikos_roomba/groundrobot.h"

GroundRobot::GroundRobot(ros::NodeHandle& n)
    : Robot(n, "GROUND ROBOT")
{
    // setup subscribers
    bumper_sub_ = n.subscribe(BUMPER_TOPIC_NAME, 10, &GroundRobot::bumperCallback, this);

    // setup services
    topSwitch_srv_ = n.advertiseService(TOPSWITCH_SERVICE_NAME, &GroundRobot::topSwitchCallback, this);

    // setup timers
    twentySec_tim_ = n.createTimer(ros::Duration(20.0), &GroundRobot::twentySecCallback, this);
    fiveSec_tim_ = n.createTimer(ros::Duration(5.0), &GroundRobot::fiveSecCallback, this);

    //twentySec_tim_.start();
    //fiveSec_tim_.start();
}

GroundRobot::~GroundRobot() {
  Robot::ROS_INFO_STREAM_ROBOT("Destruct ground robot sequence initiated.");
  // add other relevant stuff
}

void GroundRobot::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg) {
    // collision if either bumper is pressed
    if (msg->is_left_pressed || msg->is_right_pressed) {
        Robot::ROS_INFO_STREAM_ROBOT("Bumper collision");
    }
}

bool GroundRobot::topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    Robot::ROS_INFO_STREAM_ROBOT("Top switch pressed");
    Robot::ROS_INFO_STREAM_ROBOT("Roomba does a 45-deg turn");
    return true;
}

void GroundRobot::twentySecCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("20 seconds: 180 degrees");
}

void GroundRobot::fiveSecCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("5 seconds: noise");
}

void GroundRobot::updateCmbVel() {
    // simple cmdvel message for the moment
    //cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, -2.5f);
    //cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 1.0f);
}

void GroundRobot::update() {
    Robot::ROS_INFO_STREAM_ROBOT("update");

    // update timers

    updateCmbVel();

    Robot::update();
}

void GroundRobot::spinOnce()
{
  GroundRobot::update();
  ros::spinOnce();
}

void GroundRobot::spin()
{
  ros::Rate rate(loop_hz_);

  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[GROUND ROBOT] Loop running slowly.");
    }
  }
}


// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;

    GroundRobot groundrobot_(n);

    //geometry_msgs::Twist ms = robot_.getCmdVelMsg(0.0f, 2.5f);
    //robot_.publishCmdVel(ms);
    
    try
    {
        groundrobot_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[GROUND ROBOT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}