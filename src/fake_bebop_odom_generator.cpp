//
// Created by cc on 2021/12/23.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace Eigen;

nav_msgs::Odometry fake_odom_nwu;

ros::Publisher fake_odom_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    fake_odom_nwu.pose.pose.position.x = msg->pose.position.y;
    fake_odom_nwu.pose.pose.position.y = -msg->pose.position.x;
    fake_odom_nwu.pose.pose.position.z = msg->pose.position.z;

    Eigen::Quaterniond current_att;
    current_att.w() = msg->pose.orientation.w;
    current_att.x() = msg->pose.orientation.x;
    current_att.y() = msg->pose.orientation.y;
    current_att.z() = msg->pose.orientation.z;

    Eigen::Quaterniond axis;
    axis.w() = cos(M_PI/4.0);
    axis.x() = 0.0;
    axis.y() = 0.0;
    axis.z() = sin(M_PI/4.0);
    current_att = current_att * axis;

    fake_odom_nwu.pose.pose.orientation.x = current_att.x();
    fake_odom_nwu.pose.pose.orientation.y = current_att.y();
    fake_odom_nwu.pose.pose.orientation.z = current_att.z();
    fake_odom_nwu.pose.pose.orientation.w = current_att.w();

    fake_odom_pub.publish(fake_odom_nwu);
}


void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    /// ENU to NWU frame
    fake_odom_nwu.twist.twist.linear.x = msg->twist.linear.y;
    fake_odom_nwu.twist.twist.linear.y = -msg->twist.linear.x;
    fake_odom_nwu.twist.twist.linear.z = msg->twist.linear.z;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_odom_generator");

    ros::NodeHandle nh;
    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose", 1, positionCallback);
    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 1, velocityCallback);

    fake_odom_pub = nh.advertise<nav_msgs::Odometry>("/Bebop2/position_velocity_orientation_estimation", 1);

    ros::spin();
    return 0;
}
