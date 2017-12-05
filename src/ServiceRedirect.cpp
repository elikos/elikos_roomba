/**
 * \file ServiceRedirect.cpp
 * \brief ServiceRedirect class implementation
 * \author christophebedard
 */

#include "elikos_roomba/ServiceRedirect.h"

ServiceRedirect::ServiceRedirect(ros::NodeHandle& n)
    : n_(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("groundrobot_qty", groundrobotQty_);
    n_p.getParam("obstaclerobot_qty", obstaclerobotQty_);

    // setup services (global service servers)
    activate_srv_ = n_.advertiseService(ACTIVATE_SERVICE_NAME, &ServiceRedirect::activateCallback, this);
    deactivate_srv_ = n_.advertiseService(DEACTIVATE_SERVICE_NAME, &ServiceRedirect::deactivateCallback, this);
    toglActivate_srv_ = n_.advertiseService(TOGGLEACT_SERVICE_NAME, &ServiceRedirect::toglActivateCallback, this);
    reset_srv_ = n_.advertiseService(RESET_SERVICE_NAME, &ServiceRedirect::resetCallback, this);

    // setup service clients
    activate_srv_clients_ = new std::vector<ros::ServiceClient*>();
    deactivate_srv_clients_ = new std::vector<ros::ServiceClient*>();
    toglActivate_srv_clients_ = new std::vector<ros::ServiceClient*>();
    reset_srv_clients_ = new std::vector<ros::ServiceClient*>();
    createServices(GROUNDROBOT_NAMESPACE_PREFIX, groundrobotQty_);
    createServices(OBSTACLEROBOT_NAMESPACE_PREFIX, obstaclerobotQty_);
}

ServiceRedirect::~ServiceRedirect() {
    clearServiceVector(activate_srv_clients_);
    clearServiceVector(deactivate_srv_clients_);
    clearServiceVector(toglActivate_srv_clients_);
    clearServiceVector(reset_srv_clients_);
}

/*===========================
 * Callbacks
 *===========================*/

bool ServiceRedirect::activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(activate_srv_clients_);
    return true;
}

bool ServiceRedirect::deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(deactivate_srv_clients_);
    return true;
}

bool ServiceRedirect::toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(toglActivate_srv_clients_);
    return true;
}

bool ServiceRedirect::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    callServiceVector(reset_srv_clients_);
    return true;
}

/*===========================
 * Other utilities
 *===========================*/

void ServiceRedirect::createServices(std::string nsPrefix, int qty) {
    for (int i = 0; i < qty; ++i) {
        std::string robotnamespace = nsPrefix + std::to_string(i+1);
        auto act = new ros::ServiceClient(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + ACTIVATE_SERVICE_NAME));
        auto deact = new ros::ServiceClient(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + DEACTIVATE_SERVICE_NAME));
        auto togl = new ros::ServiceClient(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + TOGGLEACT_SERVICE_NAME));
        auto reset = new ros::ServiceClient(n_.serviceClient<std_srvs::Empty>(robotnamespace + "/" + RESET_SERVICE_NAME));
        activate_srv_clients_->push_back(act);
        deactivate_srv_clients_->push_back(deact);
        toglActivate_srv_clients_->push_back(togl);
        reset_srv_clients_->push_back(reset);
    }
}

void ServiceRedirect::callServiceVector(std::vector<ros::ServiceClient*>* srvs) {
    auto it = srvs->begin();
    while (it != srvs->end()) {
        (*it)->call(srv_.request, srv_.response);
        ++it;
    }
}

void ServiceRedirect::clearServiceVector(std::vector<ros::ServiceClient*>* srvs) {
    auto it = srvs->begin();
    while (it != srvs->end()) {
        delete (*it);
        ++it;
    }
    srvs->clear();
    delete srvs;
}
