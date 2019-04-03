#include <heading_hold/HHpid.h>
#include <heading_hold/heading_hold_ros.h>
#include <ros/ros.h>
#include <vortex_estimator/simple_estimator.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <tf/tf.h>
//May be included in RovState #include <geometry_msgs/pose.h>
//Constructor
HeadingHold::HeadingHold(ros::NodeHandle nh) : m_nh(nh){
    sub_estimate = m_nh.subscribe("/odometry/filtered", 1, &HeadingHold::stateEstimateCallback, this);
    sub_heading_ref = m_nh.subscribe("/yaw_reference", 1, &HeadingHold::headingReferenceCallback, this);
    pub = m_nh.advertise<vortex_msgs::PropulsionCommand>("yaw_input", 1);
    double dt = 0.1;
    //Change min and max according to Newton. Around +/- 30 N
    double max = 1.0;
    double min = -1.0;
    double K_p = 0.015;
    double K_d = 0.013;
    double K_i = 0.01;

    yaw.reset(new HHpid(dt, max, min, K_p, K_d, K_i));
}

//Destructor
HeadingHold::~HeadingHold(){}


void HeadingHold::spin()
{
    //Should change something else than Prop_command
    vortex_msgs::PropulsionCommand dh_command;
    dh_command.control_mode.resize(6);
    dh_command.control_mode[1]=1;
    pub.publish(dh_command);

    ros::Rate rate(10);
    while(ros::ok()){
        dh_command.motion[2] = -this->yaw->calculate();
        std::cout << "Heading command" << dh_command.motion[5] << std::endl;
    pub.publish(dh_command);
    ros::spinOnce();
    rate.sleep();
    }
}
void HeadingHold::headingReferenceCallback(const std_msgs::Float64 &msg){
    this->yaw_ref = msg.data;
}
void HeadingHold::stateEstimateCallback(const nav_msgs::Odometry &msg){
    //Want to find the current yaw angle
    //Orientation is given in quaternion, so
    //we wish to transfer to Euler angles
    tf::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    );

    tf::Matrix3x3 M(q);

    double roll, pitch, yaw;
    //getRPY uses pointers to get the right value to yaw
    //doesn't use roll and pitch
    M.getRPY(roll,pitch,yaw);

    double error = yaw - this->yaw_ref;

    this->yaw->updateError(error);
}