#include "elikos_roomba/groundrobot.h"

GroundRobot::GroundRobot(ros::NodeHandle& n)
    : Robot(n, GROUNDROBOT_TYPE)
{
    // setup subscribers
    bumper_sub_ = n.subscribe(BUMPER_TOPIC_NAME, 10, &GroundRobot::bumperCallback, this);

    // setup services
    topSwitch_srv_ = n.advertiseService(TOPSWITCH_SERVICE_NAME, &GroundRobot::topSwitchCallback, this);

    // setup timers
    //   oneshot FALSE, autostart TRUE
    twentySec_tim_ = n.createTimer(ros::Duration(20.0), &GroundRobot::twentySecCallback, this);
    fiveSec_tim_ = n.createTimer(ros::Duration(5.0), &GroundRobot::fiveSecCallback, this);
    //   oneshot TRUE, autostart FALSE
    bumperTurn_tim_ = n.createTimer(ros::Duration(1.0), &GroundRobot::bumperTurnTimCallback, this, true, false);
    topSwitchTurn_tim_ = n.createTimer(ros::Duration(0.25), &GroundRobot::topSwitchTurnTimCallback, this, true, false);
    timeoutTurn_tim_  = n.createTimer(ros::Duration(1.0), &GroundRobot::timeoutTurnTimCallback, this, true, false);

    // initial state
    changeRobotStateTo(INACTIVE);
    //changeRobotStateTo(FORWARD);
    forward_noise_ = 0.0;

    ROS_INFO_STREAM_ROBOT("Parent initialization done");
}

GroundRobot::~GroundRobot() {
  Robot::ROS_INFO_STREAM_ROBOT("Destruct ground robot sequence initiated.");
  // add other relevant stuff
}

void GroundRobot::changeRobotStateTo(GroundRobotState newRobotState) {
    robotState_ = newRobotState;
}
bool GroundRobot::isRobotState(GroundRobotState cmpRobotState) {
    return robotState_ == cmpRobotState;
}

void GroundRobot::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg) {
    // collision if either bumper is pressed
    if (msg->is_left_pressed || msg->is_right_pressed) {
        Robot::ROS_INFO_STREAM_ROBOT("Bumper collision");
        // if robot isn't already turning after a bumper collision
        if (!isRobotState(TURN_BUMPER)) {
            startBumperTurn();
        }
    }
}
bool GroundRobot::topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    Robot::ROS_INFO_STREAM_ROBOT("Top switch pressed");
    startTopSwitchTurn();
    return true;
}
void GroundRobot::twentySecCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("20 seconds timeout");
    startTimeoutTurn();
}
void GroundRobot::fiveSecCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("5 seconds noise timeout");
    if (isRobotState(FORWARD)) {
        // send angle noise if robot is going forward
        Robot::publishCmdVel(Robot::getCmdVelMsg(0.0f, 1.0f));
        forward_noise_ = FORWARD_NOISE;
    }
}

void GroundRobot::bumperTurnTimCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (bumper) done");
    changeRobotStateTo(FORWARD);
}
void GroundRobot::topSwitchTurnTimCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("45 degrees turn (top switch) done");
    changeRobotStateTo(FORWARD);
}
void GroundRobot::timeoutTurnTimCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (timeout) done");
    changeRobotStateTo(FORWARD);
}

void GroundRobot::startBumperTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (bumper) init");
    changeRobotStateTo(TURN_BUMPER);
    bumperTurn_tim_.stop();
    bumperTurn_tim_.setPeriod(ros::Duration(1.0));
    bumperTurn_tim_.start();
}
void GroundRobot::startTopSwitchTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("45 degrees turn (top switch) init");
    changeRobotStateTo(TURN_TOPSWITCH);
    topSwitchTurn_tim_.stop();
    topSwitchTurn_tim_.setPeriod(ros::Duration(0.25));
    topSwitchTurn_tim_.start();
}
void GroundRobot::startTimeoutTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (timeout) init");
    changeRobotStateTo(TURN_TIMEOUT);
    timeoutTurn_tim_.stop();
    timeoutTurn_tim_.setPeriod(ros::Duration(1.0));
    timeoutTurn_tim_.start();
}

void GroundRobot::updateCmbVel() {
    switch ( robotState_ ) {
        case INACTIVE:
            // nothing
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            break;
        case FORWARD:
            //cmdVel_msg_ = Robot::getCmdVelMsg(FORWARD_SPEED, 0.0f);
            cmdVel_msg_ = Robot::getCmdVelMsg(FORWARD_SPEED, forward_noise_);
            forward_noise_ = 0.0;
            break;
        case TURN_BUMPER: // 180 deg
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED);
            break;
        case TURN_TOPSWITCH: // 45 deg
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED);
            break;
        case TURN_TIMEOUT: // 180 deg
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED);
            break;
        default:
            // default behaviour
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            break;
    }
}

void GroundRobot::updateState() {
    // check global active/inactive status
    if (isReactivated_) {
        changeRobotStateTo(FORWARD);
    }
    else { // check global active/inactive status
        if (!isActive_) {
            changeRobotStateTo(INACTIVE);
        }
        // check if state is INACTIVE
        if (robotState_ == INACTIVE) {
            Robot::ROS_INFO_STREAM_ROBOT("Robot is inactive");
        }
    }
}

void GroundRobot::update() {
    //Robot::ROS_INFO_STREAM_ROBOT("update");

    updateState();
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