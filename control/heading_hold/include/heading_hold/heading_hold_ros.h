#ifndef HEADING_HOLD_ROS_H
#define HEADING_HOLD_ROS_H

#include "heading_hold/HHpid.h"
#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <nav_msgs/Odometry.h>
#include <vortex_msgs/RovState.h>
#include <std_msgs/Float64.h>



class HeadingHold
{
    private:
    ros::NodeHandle m_nh;
    ros::Publisher pub;
    ros::Subscriber sub_estimate;
    ros::Subscriber sub_heading_ref;

    double yaw_ref = 0.5;

    //Init a PID to control yaw
    std::unique_ptr<HHpid> yaw;

    public:
    HeadingHold(ros::NodeHandle m_nh);
    ~HeadingHold();
    void headingReferenceCallback(const std_msgs::Float64 &msg);
    void stateEstimateCallback(const nav_msgs::Odometry &msg);
    void spin();
};

#endif