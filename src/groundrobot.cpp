#include "elikos_roomba/groundrobot.h"

GroundRobot::GroundRobot(ros::NodeHandle& n)
    : Robot(n)
{
    // Setup services
    ros::ServiceServer topSwitch_srv_ = n.advertiseService(TOPSWITCH_SERVICE_NAME, &GroundRobot::topSwitchCallback, this);
}

GroundRobot::~GroundRobot() {
  ROS_INFO_STREAM("[GROUND ROBOT] Destruct ground robot sequence initiated.");
  // add other relevant stuff
}

bool GroundRobot::topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("[GROUND ROBOT] Roomba does a 45-deg turn");
    return true;
}

void GroundRobot::updateCmbVel() {
    // simple cmdvel message for the moment
    cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 2.5f);
}

void GroundRobot::update() {
    ROS_INFO_STREAM("[GROUND ROBOT] update");

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