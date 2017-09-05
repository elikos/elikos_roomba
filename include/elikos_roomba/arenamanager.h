#ifndef ARENA_MANAGER_H
#define ARENA_MANAGER_H

#include "elikos_roomba/service_redirect.h"        // use topic names, service names, and other values
#include "elikos_roomba/robot_viz.h"
#include <tf/transform_listener.h>


class ArenaManager
{
    private:
        tf::TransformListener tf_listener_;

        bool new_isInsideArena_[];
        bool old_isInsideArena_[];

        /*===========================
         * Services
         *===========================*/
        /* Ground robot bumper trigger service clients */
        std::vector<ros::ServiceClient> grndbot_brumpertrigger_srv_clients_;

        std_srvs::Empty srv_;
    
    protected:
        ros::NodeHandle& n_;
        double loop_hz_;
        bool is_running_slowly_;

        /* Ground robot quantity */
        int groundrobotQty_;
        /* Dimension of arena, assuming square and origin (0,0) being in the middle */
        double arenaDimension_;

        /*===========================
         * Update
         *===========================*/
        /*
         * Update; called every spinOnce()
         */
        void update();

        /*
         * ROS spin once, called on every loop
         */
        void spinOnce();
        
    public:
        /*
         * Constructor
         * groundrobotQty: number of (ground) robots
         */
        ArenaManager(ros::NodeHandle& n, int groundrobotQty, double arenaDimension);
        ~ArenaManager();

        /*
         * Wrapper for ROS_INFO_STREAM, includes node name
         */
        void ROS_INFO_STREAM_ARENAMANAGER(std::string message);

        /*
         * Concatenate string and int (because other methods weren't working)
         */
        std::string catStringInt(std::string strng, int eent);

        /*
         * Check if (x,y) coordinate is inside arena, assuming middle is (0,0)
         */
        bool isInsideArena(double x, double y);

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();
};

#endif  // ELIKOS_ROOMBA_ARENA_MANAGER_H