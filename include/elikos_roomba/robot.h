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

class Robot {
    private:


    
    protected:
        ros::NodleHandle& _n;
        ros::Publisher _cmd_vel_pub;

        RobotType _robotType;


    
    public:
        Robot(ros::NodleHandle& n);
        ~Robot();
        void update();

        // put back in protected
        geometry_msgs::Twist getCmdVelMsg();
        void publishCmdVel();

        enum RobotType {
            GROUND,
            OBSTACLE
        };
}

#endif  // ELIKOS_ROOMBA_ROBOT_H