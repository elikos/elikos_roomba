#ifndef SERVICE_REDIRECT_H
#define SERVICE_REDIRECT_H

/**
 * \file ServiceRedirect.h
 * \brief ServiceRedirect class declaration
 * \author christophebedard
 */

#include "elikos_roomba/Robot.h"        // use topic names, service names, and other values

// names
static const std::string GROUNDROBOT_NAMESPACE_PREFIX = "groundrobot";      // ground robot namespace prefix (only missing robot id)
static const std::string OBSTACLEROBOT_NAMESPACE_PREFIX = "obstaclerobot";  // obstacle robot namespace prefix (only missing robot id)

/** \class ServiceRedirect
 * \brief helper class for calling multiple services.
 */
class ServiceRedirect
{
    public:
        /**
         * \brief ServiceRedirect constructor.
         *
         * \param n : node handle.
         */
        ServiceRedirect(ros::NodeHandle& n);
        
        /**
         * \brief Robot destructor.
         */
        ~ServiceRedirect();
    
    protected:
        ros::NodeHandle& n_; /**< node handle */

        int groundrobotQty_; /**< number of target robots */
        int obstaclerobotQty_; /**< number of obstacle robots */
    
    private:
        /*===========================
         * Services
         *===========================*/
        ros::ServiceServer activate_srv_; /**< global robot activation service */
        ros::ServiceServer deactivate_srv_; /**< global robot deactivation service */
        ros::ServiceServer toglActivate_srv_; /**< global robot toggle activate service */
        ros::ServiceServer reset_srv_; /**< global robot reset service  */

        std::vector<ros::ServiceClient*>* activate_srv_clients_; /**< robot activation service clients */
        std::vector<ros::ServiceClient*>* deactivate_srv_clients_; /**< robot deactivation service clients */
        std::vector<ros::ServiceClient*>* toglActivate_srv_clients_; /**< robot toggle activate service clients */
        std::vector<ros::ServiceClient*>* reset_srv_clients_; /**< robot reset service clients */

        std_srvs::Empty srv_; /**< empty service */

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for robot activation service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for robot deactivation service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for robot toggle activate service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
         * \brief Callback class method for robot reset service.
         *
         * \param request : service request.
         * \param response : service response (empty).
         *
         * \return success.
         */
        bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Create services.
         *
         * \param nsPrefix : namespace (prefix) to use.
         * \param qty : quantity of services to create.
         */
        void createServices(std::string nsPrefix, int qty);

        /**
         * \brief Call individual services.
         *
         * \param srvs : pointer to vector of pointers to services.
         */
        void callServiceVector(std::vector<ros::ServiceClient*>* srvs);

        /**
         * \brief Deallocate services.
         *
         * \param srvs : pointer to vector of pointers to services.
         */
        void clearServiceVector(std::vector<ros::ServiceClient*>* srvs);
};

#endif  // ELIKOS_ROOMBA_SERVICE_REDIRECT_H