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

static const std::string CMDVEL_TOPIC_NAME = "cmd_vel";
static const int CMDVEL_TOPIC_QUEUESIZE = 50;

class Robot
{
    private:

    
    protected:
        ros::NodeHandle& n_;
        double loop_hz_;
        bool is_running_slowly_;

        ros::Publisher cmdVel_pub_;
        geometry_msgs::Twist cmdVel_msg_;

        void update();
        void publishCmdVel();
        void publishCmdVel(geometry_msgs::Twist cmdVel_msg);

        geometry_msgs::Twist getCmdVelMsg(float lin_x, float ang_z);
        

        //RobotType _robotType;
    
    public:
        Robot(ros::NodeHandle& n);
        ~Robot();
        virtual void spin() =0;
        virtual void spinOnce() =0;
        
};

#endif  // ELIKOS_ROOMBA_ROBOT_H