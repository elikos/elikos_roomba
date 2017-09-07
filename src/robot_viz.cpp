#include "elikos_roomba/robot_viz.h"

RobotViz::RobotViz(ros::NodeHandle& n, tf::Vector3 initial_pos, double initial_yaw, int r_id, std::string robotType)
    : n_(n),
      r_id_(r_id),
      robotType_(robotType)
{
    loop_hz_ = LOOP_RATE;

    // setup subscribers
    cmdVel_sub_ = n.subscribe(CMDVEL_TOPIC_NAME, 10, &RobotViz::cmdVelCallback, this);
    robotState_sub_ = n.subscribe(ROBOTSTATE_TOPIC_NAME, 10, &RobotViz::robotStateCallback, this);

    // setup publishers
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(ROBOTPOSE_TOPIC_NAME, ROBOTSTATE_TOPIC_QUEUESIZE);
    marker_pub_ = n.advertise<visualization_msgs::Marker>(MARKER_TOPIC_NAME, MARKER_TOPIC_QUEUESIZE);

    // create robot tf name with id and type
    tf_robot_ = catStringInt(robotType_ + TF_ROBOT_PREFIX, r_id_);

    // initial state
    isActive_ = false;

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

    ROS_INFO_STREAM_ROBOT("Initialization done (inactive)");
}

RobotViz::~RobotViz() {
  ROS_INFO_STREAM_ROBOT("Destruct sequence initiated");
  // add other relevant stuff
}

/*===========================
 * Callbacks
 *===========================*/

void RobotViz::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // check if robot is active (and time_last_ reset by robotStateCallback)
    if (isActive_) {
        // time difference
        ros::Time time_now_ = ros::Time::now();
        time_diff_ = time_now_ - time_last_;
        double timeDiffSecs = time_diff_.toSec();

        double linVel = msg->linear.x;
        double angVel = msg->angular.z;

        updatePose(timeDiffSecs, linVel, angVel);
        updatePoseMsgs();

        time_last_ = time_now_;
    }
}

void RobotViz::robotStateCallback(const std_msgs::String::ConstPtr& msg) {
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

void RobotViz::updatePose(double timeDiffSecs, double linVel, double angVel) {
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

void RobotViz::updatePoseMsgs() {
    pose_msg_ = createPoseStampedFromPosYaw(pos_, yaw_);
    tf_ = createTfFromPosYaw(pos_, yaw_);
}

void RobotViz::publishPoseMsgs() {
    pose_pub_.publish(pose_msg_);
    tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), TF_NAME_BASE, tf_robot_));
}

void RobotViz::publishMarker() {
    marker_msg_.header.stamp = ros::Time::now();
    marker_pub_.publish(marker_msg_);
}

void RobotViz::update() {
    publishPoseMsgs();
    publishMarker();
}

void RobotViz::spinOnce()
{
  update();
  ros::spinOnce();
}

void RobotViz::spin()
{
  ros::Rate rate(loop_hz_);

  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
      ROS_WARN("[ROBOT VIZ] Loop running slowly.");
    }
  }
}

/*===========================
 * Other utilities
 *===========================*/

void RobotViz::createMarkerMsg() {
    marker_msg_.header.frame_id = catStringInt("/" + robotType_ + "robot", r_id_);
    //marker_msg.ns = "myns";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg_.action = visualization_msgs::Marker::ADD;
    marker_msg_.mesh_resource = MESH_RESOURCE_PREFIX + mesh_resource_;
    marker_msg_.pose.position.x = 0.0;
    marker_msg_.pose.position.y = 0.0;
    marker_msg_.pose.position.z = 0.0;
    marker_msg_.pose.orientation.x = 0.0;
    marker_msg_.pose.orientation.y = 0.0;
    marker_msg_.pose.orientation.z = 0.0;
    marker_msg_.pose.orientation.w = 1.0;
    marker_msg_.scale.x = 1.0;
    marker_msg_.scale.y = 1.0;
    marker_msg_.scale.z = 1.0;
    // Color parameter, red by default
    ros::NodeHandle nh("~");
    std::string color = "";
    nh.getParam("robot_color", color);
    if(color == "green")
    {
        marker_msg_.color.r = 0.8f;
        marker_msg_.color.g = 1.0f;
        marker_msg_.color.b = 0.8f;
        marker_msg_.color.a = 1.0f;
    }
    else
    {
        marker_msg_.color.r = 1.0f;
        marker_msg_.color.g = 0.8f;
        marker_msg_.color.b = 0.8f;
        marker_msg_.color.a = 1.0f;
    }

    marker_msg_.lifetime = ros::Duration();
}

std::string RobotViz::generateMeshResource() {
    std::string frame_res;

    if (robotType_ == "obstacle") {
        frame_res = OBSTACLE_ROBOT_MODEL_FILE;
    } else if (robotType_ == "ground") {
        frame_res = GROUND_ROBOT_MODEL_FILE;
    }
    
    return frame_res;
}

geometry_msgs::PoseStamped RobotViz::createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = time_last_;
    pose_msg.header.frame_id = TF_NAME_BASE;
    return pose_msg;
}

tf::Transform RobotViz::createTfFromPosYaw(tf::Vector3 pos, double yaw) {
    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin(tf::Vector3(pos.x(), pos.y(), 0.0));
    q.setRPY(0, 0, yaw);
    tf.setRotation(q);
    return tf;
}

void RobotViz::ROS_INFO_STREAM_ROBOT(std::string message) {
    ROS_INFO_STREAM("[" << "ROBOT VIZ " << robotType_ << "robot" << r_id_ << "] " << message);
}

std::string RobotViz::catStringInt(std::string strng, int eent) {
    std::stringstream sstm;
    sstm << strng << eent;
    return sstm.str();
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotviz");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    double init_pos_x, init_pos_y, init_pos_z, init_yaw;
    int robot_id;
    std::string robot_type;
    n_p.getParam("init_pos_x", init_pos_x);
    n_p.getParam("init_pos_y", init_pos_y);
    n_p.getParam("init_pos_z", init_pos_z);
    n_p.getParam("init_yaw", init_yaw);
    n_p.getParam("robot_id", robot_id);
    n_p.getParam("robot_type", robot_type);

    RobotViz robotviz_(n, tf::Vector3(init_pos_x, init_pos_y, init_pos_z), init_yaw, robot_id, robot_type);
    
    try
    {
        robotviz_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ROBOT VIZ] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
