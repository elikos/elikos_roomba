#include "elikos_roomba/service_redirect.h"


ServiceRedirect::ServiceRedirect(ros::NodeHandle& n, int robotQty)
    : n_(n),
      robotQty_(robotQty)
{
    // setup services (global service servers)
    activate_srv_ = n_.advertiseService(ACTIVATE_SERVICE_NAME, &ServiceRedirect::activateCallback, this);
    deactivate_srv_ = n_.advertiseService(DEACTIVATE_SERVICE_NAME, &ServiceRedirect::deactivateCallback, this);
    toglActivate_srv_ = n_.advertiseService(TOGGLEACT_SERVICE_NAME, &ServiceRedirect::toglActivateCallback, this);

    // setup service clients
    for (int i = 0; i < robotQty_; ++i) {
        std::string robotnamespace = catStringInt(ROBOT_NAMESPACE_PREFIX, i+1);
        activate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + ACTIVATE_SERVICE_NAME));
        deactivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + DEACTIVATE_SERVICE_NAME));
        toglActivate_srv_clients_.push_back(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + TOGGLEACT_SERVICE_NAME));
    }
}

ServiceRedirect::~ServiceRedirect() {
  // add other relevant stuff
}

/*===========================
 * Callbacks
 *===========================*/

bool ServiceRedirect::activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&activate_srv_clients_);
    return true;
}

bool ServiceRedirect::deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&deactivate_srv_clients_);
    return true;
}

bool ServiceRedirect::toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(&toglActivate_srv_clients_);
    return true;
}

/*===========================
 * Other utilities
 *===========================*/

 void ServiceRedirect::callServiceVector(std::vector<ros::ServiceClient>* srv_clients) {
    for (int i = 0; i < robotQty_; ++i) {
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
    int robotQty;
    n_p.getParam("robot_qty", robotQty);

    ServiceRedirect serviceredirect_(n, robotQty);
    
    try
    {
        ros::spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[ROBOT VIZ] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
