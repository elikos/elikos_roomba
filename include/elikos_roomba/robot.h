#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

class Robot
{
    private:


    
    protected:
        ros::NodeHandle& n_;
        ros::Publisher cmd_vel_pub_;

        //RobotType _robotType;


    
    public:
        Robot(ros::NodeHandle& n);
        ~Robot();
        void update();

        // put back in protected
        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);

        enum RobotType {
            GROUND,
            OBSTACLE
        };
};

#endif  // ELIKOS_ROOMBA_ROBOT_H