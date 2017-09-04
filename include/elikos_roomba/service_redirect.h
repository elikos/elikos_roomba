#ifndef SERVICE_REDIRECT_H
#define SERVICE_REDIRECT_H

#include "elikos_roomba/robot.h"        // use topic names, service names, and other values

// names
static const std::string GROUNDROBOT_NAMESPACE_PREFIX = "groundrobot";      // ground robot namespace prefix (only missing robot id)
static const std::string OBSTACLEROBOT_NAMESPACE_PREFIX = "obstaclerobot";  // obstacle robot namespace prefix (only missing robot id)


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

        /* Ground robot activation service clients */
        std::vector<ros::ServiceClient> grndbot_activate_srv_clients_;
        /* Ground robot deactivation service clients */
        std::vector<ros::ServiceClient> grndbot_deactivate_srv_clients_;
        /* Ground robot toggle activate service clients */
        std::vector<ros::ServiceClient> grndbot_toglActivate_srv_clients_;

        /* Obstacle robot activation service clients */
        std::vector<ros::ServiceClient> obsbot_activate_srv_clients_;
        /* Obstacle robot deactivation service clients */
        std::vector<ros::ServiceClient> obsbot_deactivate_srv_clients_;
        /* Obstacle robot toggle activate service clients */
        std::vector<ros::ServiceClient> obsbot_toglActivate_srv_clients_;

        std_srvs::Empty srv_;
    
    protected:
        ros::NodeHandle& n_;

        int groundrobotQty_;
        int obstaclerobotQty_;

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
         * groundrobotQty: number of ground robots
         * obstaclerobotQty: number of obstacle robots
         */
        ServiceRedirect(ros::NodeHandle& n, int groundrobotQty, int obstaclerobotQty);
        ~ServiceRedirect();

        /*
         * Call individual services in vector corresponding to pointer
         */
        void callServiceVector(std::vector<ros::ServiceClient>* srv_clients, int botQty);

        /*
         * Concatenate string and int (because other methods weren't working)
         */
        std::string catStringInt(std::string strng, int eent);
};

#endif  // ELIKOS_ROOMBA_SERVICE_REDIRECT_H