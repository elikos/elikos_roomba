#include "elikos_roomba/arenamanager.h"


ArenaManager::ArenaManager(ros::NodeHandle& n, int groundrobotQty, double arenaDimension)
    : n_(n),
      groundrobotQty_(groundrobotQty),
      arenaDimension_(arenaDimension)
{
    loop_hz_ = 20.0;

    bool new_isInsideArena_[groundrobotQty_] = {true};
    bool old_isInsideArena_[groundrobotQty_] = {true};
    //bool new_isInsideArena_[groundrobotQty_];
    //bool old_isInsideArena_[groundrobotQty_];

    // setup service clients for ground robot bumper trigger
    for (int i = 0; i < groundrobotQty_; ++i) {
        std::string robotnamespace = catStringInt(GROUNDROBOT_NAMESPACE_PREFIX, i+1);
        grndbot_brumpertrigger_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + BUMPER_SERVICE_NAME));
        //old_isInsideArena_[i] = true;
        //new_isInsideArena_[i] = true;
    }

    tf_listener_.waitForTransform("/" + catStringInt(GROUNDROBOT_NAMESPACE_PREFIX, 1), TF_NAME_BASE, ros::Time::now(), ros::Duration(2.0));
}

ArenaManager::~ArenaManager() {
  // add other relevant stuff
}

/*===========================
 * Other utilities
 *===========================*/

void ArenaManager::ROS_INFO_STREAM_ARENAMANAGER(std::string message) {
    ROS_INFO_STREAM("[ARENA MANAGER] " << message);
}

std::string ArenaManager::catStringInt(std::string strng, int eent) {
    std::stringstream sstm;
    sstm << strng << eent;
    return sstm.str();
}

bool ArenaManager::isInsideArena(double x, double y) {
    return (x >= -arenaDimension_/2) && (x <= arenaDimension_/2) && (y >= -arenaDimension_/2) && (y <= arenaDimension_/2);
}

/*===========================
 * Update
 *===========================*/

void ArenaManager::update() {
    tf::StampedTransform transform;
    for (int i = 0; i < groundrobotQty_; ++i) {
        // listen to tf
        tf_listener_.lookupTransform("/" + catStringInt(GROUNDROBOT_NAMESPACE_PREFIX, i+1), TF_NAME_BASE, ros::Time(0), transform);
        new_isInsideArena_[i] = isInsideArena(transform.getOrigin().x(), transform.getOrigin().y());
        ROS_INFO_STREAM_ARENAMANAGER(catStringInt("Ground robot ", i+1));
        if (new_isInsideArena_[i]) {
            ROS_INFO_STREAM_ARENAMANAGER("is inside arena");
        } else {
            ROS_INFO_STREAM_ARENAMANAGER("is NOT inside arena");
        }

        if (old_isInsideArena_[i]) {
            ROS_INFO_STREAM_ARENAMANAGER("was inside arena");
        } else {
            ROS_INFO_STREAM_ARENAMANAGER("was NOT inside arena");
        }

        // if status just changed to being outside arena, trigger turn
        if (!new_isInsideArena_[i] && old_isInsideArena_[i]) {
            ROS_INFO_STREAM_ARENAMANAGER(catStringInt("Ground robot ", i+1) + " is outside arena; triggering bumper");
            grndbot_brumpertrigger_srv_clients_[i].call(srv_.request, srv_.response);
        }

        old_isInsideArena_[i] = new_isInsideArena_[i];
    }
}

void ArenaManager::spinOnce()
{
    update();
    ros::spinOnce();
}

void ArenaManager::spin()
{
    ros::Rate rate(loop_hz_);
    
    while (ros::ok())
    {
        spinOnce();
        
        is_running_slowly_ = !rate.sleep();
        if (is_running_slowly_)
        {
            ROS_WARN("[ARENA MANAGER] Loop running slowly.");
        }
    }
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arenamanager");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    int groundrobotQty;
    double arenaDimension;
    n_p.getParam("groundrobot_qty", groundrobotQty);
    n_p.getParam("arena_dimension", arenaDimension);

    ArenaManager arenamanager_(n, groundrobotQty, arenaDimension);
    
    try
    {
        arenamanager_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ARENA MANAGER] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
