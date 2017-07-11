#ifndef SERVICE_REDIRECT_H
#define SERVICE_REDIRECT_H

#include "elikos_roomba/robot.h"        // use topic names, service names, and other values

// names
static const std::string ROBOT_NAMESPACE_PREFIX = "robot";      // robot namespace prefix (only missing robot id)


class ServiceRedirect
{
    private:
        /*===========================
         * Services
         *===========================*/
        /* Global robot activation service  */
        ros::ServiceServer activate_srv_;
        /* Global robot deactivation service */
        ros::ServiceServer deactivate_srv_;
        /* Global robot toggle activate service */
        ros::ServiceServer toglActivate_srv_;

        /* Robot activation service clients */
        std::vector<ros::ServiceClient> activate_srv_clients_;
        /* Robot deactivation service clients */
        std::vector<ros::ServiceClient> deactivate_srv_clients_;
        /* Robot toggle activate service clients */
        std::vector<ros::ServiceClient> toglActivate_srv_clients_;

        std_srvs::Empty srv_;
    
    protected:
        ros::NodeHandle& n_;

        int robotQty_;

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for robot activation service
         */
        bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Callback class method for robot deactivation service
         */
        bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /*
         * Callback class method for robot toggle activate service
         */
        bool toglActivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
    public:
        /*
         * Constructor
         * robotQty: number of robots
         */
        ServiceRedirect(ros::NodeHandle& n, int robotQty);
        ~ServiceRedirect();

        /*
         * Call individual services in vector corresponding to pointer
         */
        void callServiceVector(std::vector<ros::ServiceClient>* srv_clients);

        /*
         * Concatenate string and int (because other methods weren't working)
         */
        std::string catStringInt(std::string strng, int eent);
};

#endif  // ELIKOS_ROOMBA_SERVICE_REDIRECT_H