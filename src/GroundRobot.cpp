#include "elikos_roomba/GroundRobot.h"

GroundRobot::GroundRobot(ros::NodeHandle& n, int r_id, tf::Vector3 initial_pos, double initial_yaw, std::string color)
    : Robot(n, GROUNDROBOT_TYPE, r_id, initial_pos, initial_yaw, color)
{
    // setup subscribers
    bumper_sub_ = n.subscribe(ns_ + "/" + BUMPER_TOPIC_NAME, 10, &GroundRobot::bumperCallback, this);

    // setup services
    topSwitch_srv_ = n.advertiseService(ns_ + "/" + TOPSWITCH_SERVICE_NAME, &GroundRobot::topSwitchCallback, this);
    bumper_srv_ = n.advertiseService(ns_ + "/" + BUMPER_SERVICE_NAME, &GroundRobot::bumperTrigCallback, this);

    // setup timers (oneshot TRUE, autostart FALSE)
    timeout_tim_ = n.createTimer(ros::Duration(TIMEOUT_DURATION), &GroundRobot::timeoutCallback, this, true, false);
    noise_tim_ = n.createTimer(ros::Duration(NOISE_DURATION), &GroundRobot::noiseCallback, this, true, false);
    noiseTurn_tim_ = n.createTimer(ros::Duration(getTurnDurationFromAngleAndSpeed(0.0,NOISE_TURN_SPEED)), &GroundRobot::noiseTurnCallback, this, true, false);
    bumperTurn_tim_ = n.createTimer(ros::Duration(BUMPER_TURN_DURATION), &GroundRobot::bumperTurnTimCallback, this, true, false);
    topSwitchTurn_tim_ = n.createTimer(ros::Duration(TOPSWITCH_TURN_DURATION), &GroundRobot::topSwitchTurnTimCallback, this, true, false);
    timeoutTurn_tim_  = n.createTimer(ros::Duration(TIMEOUT_TURN_DURATION), &GroundRobot::timeoutTurnTimCallback, this, true, false);

    // initial state
    changeRobotStateTo(INACTIVE);
    forward_noise_ = 0.0;

    //ROS_INFO_STREAM_ROBOT("Parent initialization done");

    // seed rand()
    std::srand(ros::WallTime::now().toNSec());
}

GroundRobot::~GroundRobot() {
    Robot::ROS_INFO_STREAM_ROBOT("Destruct ground robot sequence initiated");
    // add other relevant stuff
}

/*===========================
 * Ground robot state changes
 *===========================*/

void GroundRobot::changeRobotStateTo(GroundRobotState newRobotState) {
    robotState_ = newRobotState;

}

bool GroundRobot::isRobotState(GroundRobotState cmpRobotState) {
    return robotState_ == cmpRobotState;
}

/*===========================
 * Global state
 *===========================*/

void GroundRobot::activateRobot() {
    //ROS_INFO_STREAM_ROBOT("Parent robot activated");
    changeRobotStateTo(FORWARD);
    // reactivate timers (timeout and noise)
    timerRestart(timeout_tim_, TIMEOUT_DURATION);
    timerRestart(noise_tim_, NOISE_DURATION);
    Robot::activateRobot();
}

void GroundRobot::deactivateRobot() {
    //ROS_INFO_STREAM_ROBOT("Parent robot deactivated");
    changeRobotStateTo(INACTIVE);
    // deactivate timers (all)
    timeout_tim_.stop();
    noise_tim_.stop();
    noiseTurn_tim_.stop();
    bumperTurn_tim_.stop();
    topSwitchTurn_tim_.stop();
    timeoutTurn_tim_.stop();
    Robot::deactivateRobot();
}

/*===========================
 * Other utilities
 *===========================*/

double GroundRobot::getRandomNoiseAngle() {
    return NOISE_ANGLE_MIN + ((double)std::rand() / RAND_MAX) * (NOISE_ANGLE_MAX - NOISE_ANGLE_MIN);
}

double GroundRobot::getTurnDurationFromAngleAndSpeed(double angl, double speed) {
    return angl/speed;
}

void GroundRobot::timerRestart(ros::Timer& tim, double dur) {
    tim.stop();
    tim.setPeriod(ros::Duration(dur));
    tim.start();
}

/*===========================
 * Callbacks
 *===========================*/

void GroundRobot::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg) {
    // collision if either bumper is pressed
    if (msg->is_left_pressed || msg->is_right_pressed) {
        Robot::ROS_INFO_STREAM_ROBOT("Bumper collision");
        bumperTrig();
    }
}

bool GroundRobot::bumperTrigCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    bumperTrig();
    return true;
}

void GroundRobot::bumperTrig() {
    Robot::ROS_INFO_STREAM_ROBOT("Bumper triggered");
    // if robot is going forward
    if (isRobotState(FORWARD)) {
        startBumperTurn();
    }
}

void GroundRobot::topswitchTrig() {
    Robot::ROS_INFO_STREAM_ROBOT("Topswitch triggered");
    // if robot is going forward
    if (isRobotState(FORWARD)) {
        startTopSwitchTurn();
    }
}

bool GroundRobot::topSwitchCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    Robot::ROS_INFO_STREAM_ROBOT("Top switch pressed");
    topswitchTrig();
    return true;
}

void GroundRobot::timeoutCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("20 seconds timeout");
    startTimeoutTurn();
    timerRestart(timeout_tim_, TIMEOUT_DURATION);
}

void GroundRobot::noiseCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("5 seconds noise timeout");
    // if robot is going forward
    if (isRobotState(FORWARD)) {
        // start to turn (as noise)
        double randAngle = getRandomNoiseAngle();
        std::ostringstream strs;
        strs << (randAngle/DEG_TO_RAD);
        Robot::ROS_INFO_STREAM_ROBOT("Starting to add noise (" + strs.str() + " degrees)");
        forward_noise_ = NOISE_TURN_SPEED;
        timerRestart(noiseTurn_tim_, getTurnDurationFromAngleAndSpeed(randAngle,NOISE_TURN_SPEED));;
    }
    else { // or else restart the timer
        timerRestart(noise_tim_, NOISE_DURATION);
    }
}

void GroundRobot::noiseTurnCallback(const ros::TimerEvent& event) {
    Robot::ROS_INFO_STREAM_ROBOT("Noise done");
    forward_noise_ = 0.0f; // set to 0
    timerRestart(noise_tim_, NOISE_DURATION);
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

/*===========================
 * Actions
 *===========================*/

void GroundRobot::startBumperTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (bumper) init");
    changeRobotStateTo(TURN_BUMPER);
    timerRestart(bumperTurn_tim_, BUMPER_TURN_DURATION);
}

void GroundRobot::startTopSwitchTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("45 degrees turn (top switch) init");
    changeRobotStateTo(TURN_TOPSWITCH);
    timerRestart(topSwitchTurn_tim_, TOPSWITCH_TURN_DURATION);
}

void GroundRobot::startTimeoutTurn() {
    Robot::ROS_INFO_STREAM_ROBOT("180 degrees turn (timeout) init");
    changeRobotStateTo(TURN_TIMEOUT);
    timerRestart(timeoutTurn_tim_, TIMEOUT_TURN_DURATION);
}

/*===========================
 * Update
 *===========================*/

void GroundRobot::checkTopInteraction(tf::Vector3 pos, double diameter) {
    double distanceSquaredWrtMe_xy = pow((pos.getY() - pos_.getY()), 2) + pow((pos.getX() - pos_.getX()), 2);
    double distanceWrtMe_z = pos.getZ() - pos_.getZ();

    bool isWithinRangeXY = distanceSquaredWrtMe_xy <= pow(DIAMETER/2.0 + diameter/2.0, 2);
    bool isTouchingZ = distanceWrtMe_z <= HEIGHT;

    //if interaction
    if (isWithinRangeXY && isTouchingZ) {
        topswitchTrig();
    }
}

void GroundRobot::checkRobotCollision(tf::Vector3 pos) {
    // distance
    double distanceSquaredWrtMe = pow((pos.getY() - pos_.getY()), 2) + pow((pos.getX() - pos_.getX()), 2);

    // yaw
    double angle = atan2(pos.getY() - pos_.getY(), pos.getX() - pos_.getX());
    // actual yaw : difference between my heading and the robot's angle wrt me
    double yaw_diff = angle - yaw_;

    bool isTooClose = distanceSquaredWrtMe <= pow(DIAMETER, 2);
    bool isBumper = (yaw_diff >= (-BUMPER_ANGLE/2)) && (yaw_diff <= (BUMPER_ANGLE/2));

    // if collision
    if (isTooClose && isBumper) {
        bumperTrig();
    }
}

void GroundRobot::updateState() {
    // check reset state
    if (isReset_) {
        deactivateRobot();
    }

    // set cmd_vel msg according to state
    switch ( robotState_ ) {
        case INACTIVE:
            // nothing
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            robotState_msg_.data = "INACTIVE";
            break;
        case FORWARD:
            cmdVel_msg_ = Robot::getCmdVelMsg(FORWARD_SPEED, forward_noise_*ROTATE_CCW);
            robotState_msg_.data = "FORWARD";
            break;
        case TURN_BUMPER:
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED*ROTATE_CW);
            robotState_msg_.data = "TURN_BUMPER";
            break;
        case TURN_TOPSWITCH:
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED*ROTATE_CW);
            robotState_msg_.data = "TURN_TOPSWITCH";
            break;
        case TURN_TIMEOUT:
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, TURN_SPEED*ROTATE_CW);
            robotState_msg_.data = "TURN_TIMEOUT";
            break;
        default:
            // default behaviour
            cmdVel_msg_ = Robot::getCmdVelMsg(0.0f, 0.0f);
            robotState_msg_.data = "INACTIVE";
            break;
    }
}

void GroundRobot::update() {
    //Robot::ROS_INFO_STREAM_ROBOT("update");
    updateState();

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
/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundrobot");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    double init_pos_x, init_pos_y, init_pos_z, init_yaw;
    int robot_id;
    std::string robot_color;
    n_p.getParam("init_pos_x", init_pos_x);
    n_p.getParam("init_pos_y", init_pos_y);
    n_p.getParam("init_pos_z", init_pos_z);
    n_p.getParam("init_yaw", init_yaw);
    n_p.getParam("robot_id", robot_id);
    n_p.getParam("robot_color", robot_color);

    GroundRobot groundrobot_(n, robot_id, tf::Vector3(init_pos_x, init_pos_y, init_pos_z), init_yaw, robot_color);
    
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
*/