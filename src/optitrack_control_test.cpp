//
// Created by cc on 2020/8/5.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub, current_marker_pub;

double take_off_height = 1.0;


//void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
//{
//    /// NWU
//    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//}


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    /// NWU
    current_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void marker_publish(const Eigen::MatrixXd &Points, int id, float r, float g, float b, float width, int publish_num, int type)
{
//    uint8 ARROW=0//箭头
//    uint8 CUBE=1//立方体
//    uint8 SPHERE=2//球
//    uint8 CYLINDER=3//圆柱体
//    uint8 LINE_STRIP=4//线条（点的连线）
//    uint8 LINE_LIST=5//线条序列
//    uint8 CUBE_LIST=6//立方体序列
//    uint8 SPHERE_LIST=7//球序列
//    uint8 POINTS=8//点集
//    uint8 TEXT_VIEW_FACING=9//显示3D的文字
//    uint8 MESH_RESOURCE=10//网格？
//    uint8 TRIANGLE_LIST=11//三角形序列

    if(Points.rows() < 1) return;

    visualization_msgs::Marker points;
    points.header.frame_id = "world";
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;
    points.ns = "lines_and_points";
    points.id = id;
    points.type = type;

    // Line width
    points.scale.x = width;
    points.scale.y = width;

    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;
    points.lifetime = ros::Duration(0);

    int step = 1;
    if(publish_num > 0){
        step = (int)Points.rows() / publish_num;
    }

    for(int i=0; i<Points.rows(); i+=step)
    {
        geometry_msgs::Point p;
        p.x = Points(i, 0);
        p.y = Points(i, 1);
        p.z = Points(i, 2);

        points.points.push_back(p);
    }

    geometry_msgs::Point p;
    p.x = Points(Points.rows()-1, 0);
    p.y = Points(Points.rows()-1, 1);
    p.z = Points(Points.rows()-1, 2);
    points.points.push_back(p);

    current_marker_pub.publish(points);
}


void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));

    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));

    pva_pub.publish(pva_setpoint);
}


/** This function is to generate state to state trajectory **/
void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, double v_max, double delt_t,
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    // % Choose the time as running in average velocity
    // double decay_parameter = 0.5;
    // double T = 0.2;

    double j_limit = 5;
    double a_limit = 3;
    double v_limit = v_max;

//    double T1 = fabs(af(0)-a0(0))/j_limit > fabs(af(1)-a0(1))/j_limit ? fabs(af(0)-a0(0))/j_limit : fabs(af(1)-a0(1))/j_limit;
//    T1 = T1 > fabs(af(2)-a0(2))/j_limit ? T1 : fabs(af(2)-a0(2))/j_limit;
    double T2 = fabs(vf(0)-v0(0))/a_limit > fabs(vf(1)-v0(1))/a_limit ? fabs(vf(0)-v0(0))/a_limit : fabs(vf(1)-v0(1))/a_limit;
    T2 = T2 > fabs(vf(2)-v0(2))/a_limit ? T2 : fabs(vf(2)-v0(2))/a_limit;
    double T3 = fabs(pf(0)-p0(0))/v_limit > fabs(pf(1)-p0(1))/v_limit ? fabs(pf(0)-p0(0))/v_limit : fabs(pf(1)-p0(1))/v_limit;
    T3 = T3 > fabs(pf(2)-p0(2))/v_limit ? T3 : fabs(pf(2)-p0(2))/v_limit;

//    double T = T1 > T2 ? T1 : T2;
//    T = T > T3 ? T : T3;
    double T = T2;
    T = T > T3 ? T : T3;
    T = T < 0.3 ? 0.3 : T;

    T *= 3.0; // slow down fator

//    ROS_INFO_THROTTLE(2, "T=%lf", T);

    int times = T / delt_t;

    p = Eigen::MatrixXd::Zero(times, 3);
    v = Eigen::MatrixXd::Zero(times, 3);
    a = Eigen::MatrixXd::Zero(times, 3);
    t = Eigen::VectorXd::Zero(times);

    // % calculate optimal jerk controls by Mark W. Miller
    for(int ii=0; ii<3; ii++)
    {
        double delt_a = af(ii) - a0(ii);
        double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
        double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

        //%  if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

        for(int jj=0; jj<times; jj++)
        {
            double tt = (jj + 1)*delt_t;
            t(jj) = tt;
            p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
            v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
            a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);

            /// NOTE: Let z be constant here
            p(jj,2) = take_off_height;
            v(jj,2) = 0;
            a(jj,2) = 0;
        }

        
    }
}

void compute_circular_traj(const double r, const double vel, const Eigen::Vector3d p0, const double t,
                           Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
//@requires r > 0 && vel > 0 && t >= 0;
{
    const double theta = vel*t/r;

    p(0) = r*cos(theta) + p0(0) - r;
    p(1) = r*sin(theta) + p0(1);
    p(2) = p0(2);

    v(0) = -vel*sin(theta);
    v(1) = vel*cos(theta);
    v(2) = 0;

    a(0) = -vel*vel/r*cos(theta);
    a(1) = -vel*vel/r*sin(theta);
    a(2) = 0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "nokov_control_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
//    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/drone/pose", 1, positionCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Bebop2/position_velocity_orientation_estimation", 1, odomCallback);

    current_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);


    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);


    const int LOOPRATE = 40;
    ros::Rate loop_rate(LOOPRATE);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    double yaw_target = 0.0;
    while(current_state.mode != "OFFBOARD"){
        setPVA(current_p, Vector3d::Zero(), Vector3d::Zero(), yaw_target);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Hover for 3 seconds
    int hover_counter = 0;
    Vector3d recorded_p = current_p;

    ROS_INFO("Hovering");
    while(hover_counter < 120){
        setPVA(recorded_p, Vector3d::Zero(), Vector3d::Zero(), yaw_target);
        hover_counter ++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    double delt_t = 1.0 / LOOPRATE;

    MatrixXd p_t, v_t, a_t;
    Eigen::VectorXd t_vector;
    Vector3d v0(0.0, 0.0, 0.0);
    Vector3d a0(0.0, 0.0, 0.0);

    // Vector3d pf(circle_radius, 0, take_off_height);
    Vector3d pf(3, 0, take_off_height);
    Vector3d vf(0, 0, 0);
    Vector3d af(0, 0, 0);

    ROS_INFO("current_p = (%f, %f, %f), current_v = (%f, %f, %f), current_a = (%f, %f, %f), p_f = (%f, %f, %f)", current_p(0), current_p(1), current_p(2),
    v0(0), v0(1), v0(2), a0(0), a0(1), a0(2), pf(0), pf(1), pf(2));

    motion_primitives(current_p, v0, a0, pf, vf, af, 3.0, delt_t, p_t, v_t, a_t, t_vector);
    ROS_INFO("Minimum jerk trajectory planned!");

    marker_publish(p_t, 199, 1, 0, 0, 0.1, -1, 8);
    ros::Duration(0.01).sleep();
    marker_publish(p_t, 199, 1, 0, 0, 0.1, -1, 8);
    ros::Duration(0.01).sleep();
    marker_publish(p_t, 199, 1, 0, 0, 0.1, -1, 8);

    for(int i=0; i<t_vector.size(); i++)
    {
        setPVA(p_t.row(i), v_t.row(i), a_t.row(i));
        loop_rate.sleep();
        ros::spinOnce();
    }

    // Hover
    ROS_INFO("Hovering");

    while(ros::ok())
    {
        setPVA(p_t.row(t_vector.size()-1), v_t.row(t_vector.size()-1), Vector3d::Zero());// a_t.row(i));
        Vector3d last_sp_p = p_t.row(t_vector.size()-1);
        Vector3d delt_p = last_sp_p - current_p;

        if(delt_p.norm() < 0.2){
            ROS_WARN_THROTTLE(3, "Align Complete!");
            // break;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
