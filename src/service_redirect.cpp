#include "elikos_roomba/service_redirect.h"


ServiceRedirect::ServiceRedirect(ros::NodeHandle& n, int groundrobotQty, int obstaclerobotQty)
    : n_(n),
      groundrobotQty_(groundrobotQty),
      obstaclerobotQty_(obstaclerobotQty)
{
    // setup services (global service servers)
    activate_srv_ = n_.advertiseService(ACTIVATE_SERVICE_NAME, &ServiceRedirect::activateCallback, this);
    deactivate_srv_ = n_.advertiseService(DEACTIVATE_SERVICE_NAME, &ServiceRedirect::deactivateCallback, this);
    toglActivate_srv_ = n_.advertiseService(TOGGLEACT_SERVICE_NAME, &ServiceRedirect::toglActivateCallback, this);

    // setup service clients for ground robots
    for (int i = 0; i < groundrobotQty_; ++i) {
        std::string robotnamespace = catStringInt(GROUNDROBOT_NAMESPACE_PREFIX, i+1);
        grndbot_activate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + ACTIVATE_SERVICE_NAME));
        grndbot_deactivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + DEACTIVATE_SERVICE_NAME));
        grndbot_toglActivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + TOGGLEACT_SERVICE_NAME));
    }

    // setup service clients for obstacle robots
    for (int i = 0; i < obstaclerobotQty_; ++i) {
        std::string robotnamespace = catStringInt(OBSTACLEROBOT_NAMESPACE_PREFIX, i+1);
        obsbot_activate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + ACTIVATE_SERVICE_NAME));
        obsbot_deactivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + DEACTIVATE_SERVICE_NAME));
        obsbot_toglActivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + TOGGLEACT_SERVICE_NAME));
    }
}

ServiceRedirect::~ServiceRedirect() {
  // add other relevant stuff
}

/*===========================
 * Callbacks
 *===========================*/

bool ServiceRedirect::activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&grndbot_activate_srv_clients_, groundrobotQty_);
    callServiceVector(&obsbot_activate_srv_clients_, obstaclerobotQty_);
    return true;
}

bool ServiceRedirect::deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&grndbot_deactivate_srv_clients_, groundrobotQty_);
    callServiceVector(&obsbot_deactivate_srv_clients_, obstaclerobotQty_);
    return true;
}

bool ServiceRedirect::toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&grndbot_toglActivate_srv_clients_, groundrobotQty_);
    callServiceVector(&obsbot_toglActivate_srv_clients_, obstaclerobotQty_);
    return true;
}

/*===========================
 * Other utilities
 *===========================*/

 void ServiceRedirect::callServiceVector(std::vector<ros::ServiceClient>* srv_clients, int botQty) {
    for (int i = 0; i < botQty; ++i) {
        srv_clients->operator[](i).call(srv_.request, srv_.response);
    }
}

std::string ServiceRedirect::catStringInt(std::string strng, int eent) {
    std::stringstream sstm;
    sstm << strng << eent;
    return sstm.str();
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serviceredirect");
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    int groundrobotQty, obstaclerobotQty;
    n_p.getParam("groundrobot_qty", groundrobotQty);
    n_p.getParam("obstaclerobot_qty", obstaclerobotQty);

    ServiceRedirect serviceredirect_(n, groundrobotQty, obstaclerobotQty);
    
    try
    {
        ros::spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[SERVICE REDIRECT] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
