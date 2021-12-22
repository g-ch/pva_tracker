//
// Created by cc on 2020/8/4.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <pva_tracker/PVA_TrackerConfig.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#define GRAVITATIONAL_ACC 9.81

mavros_msgs::State current_state;

using namespace Eigen;

// Coefficients
Vector3d position_error_p;
Vector3d position_error_ff;
Vector3d position_error_d;
Vector3d position_error_i;
Vector3d velocity_error_p;
Vector3d velocity_error_d;
Vector3d velocity_error_i;

double p_i_acc_error_limit;
double v_i_acc_error_limit;

// Global Variables
Vector3d planned_p;
Vector3d planned_v;
Vector3d planned_a;
double planned_yaw;
Vector3d current_p;
Vector3d current_v;
Quaterniond current_att;
ros::Publisher att_ctrl_pub, odom_sp_enu_pub, pose_estimation_pub;
double thrust_factor, thrust_factor_max, thrust_factor_min;
double max_flight_time, max_stand_by_time;
double flight_time_second = 0.0, stand_by_time_second = 0.0;
bool pva_command_received = false;


Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}

void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i=0; i<3; i++){
            v(i) = fabs(v(i)) > limit ? (v(i) > 0 ? limit : -limit) : v(i);
        }
    }
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}


double last_time = 0.0;
bool time_counter_initialized = false;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    /// NWU to ENU frame
    current_p << -msg->pose.pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.z;
    current_att.w() = msg->pose.pose.orientation.w;
    current_att.x() = msg->pose.pose.orientation.x;
    current_att.y() = msg->pose.pose.orientation.y;
    current_att.z() = msg->pose.pose.orientation.z;

    Eigen::Quaterniond axis;
    axis.w() = cos(-M_PI/4.0);
    axis.x() = 0.0;
    axis.y() = 0.0;
    axis.z() = sin(-M_PI/4.0);
    current_att = current_att * axis;

    geometry_msgs::PoseStamped pose_to_send;
    pose_to_send.header.stamp = ros::Time::now();
    pose_to_send.pose.position.x = current_p.x();
    pose_to_send.pose.position.y = current_p.y();
    pose_to_send.pose.position.z = current_p.z();
    pose_to_send.pose.orientation.w = current_att.w();
    pose_to_send.pose.orientation.x = current_att.x();
    pose_to_send.pose.orientation.y = current_att.y();
    pose_to_send.pose.orientation.z = current_att.z();

    pose_estimation_pub.publish(pose_to_send);


    if(!time_counter_initialized){
        last_time = ros::Time::now().toSec();
        time_counter_initialized = true;
    }else{
        if(current_state.armed && msg->pose.pose.position.z > 0.2){
            flight_time_second += ros::Time::now().toSec() - last_time;
        }else{
            stand_by_time_second += ros::Time::now().toSec() - last_time;
        }
        last_time = ros::Time::now().toSec();
        double coeff_stand_by_time_to_flight_time = max_flight_time / max_stand_by_time;
        thrust_factor = thrust_factor_min + (thrust_factor_max-thrust_factor_min)*(flight_time_second / 60.0 + stand_by_time_second / 60.0*coeff_stand_by_time_to_flight_time) / max_flight_time;
        if(thrust_factor > thrust_factor_max) thrust_factor = thrust_factor_max;
    }

    current_v << -msg->twist.twist.linear.y, msg->twist.twist.linear.x, msg->twist.twist.linear.z;
}


void pvaPointSet(const trajectory_msgs::JointTrajectoryPoint& msg){
    mavros_msgs::AttitudeTarget att_setpoint;

    /// NWU frame to ENU frame
    planned_p << -msg.positions[1], msg.positions[0], msg.positions[2];
    planned_yaw = msg.positions[3];// + M_PI/2.0;
    planned_v << -msg.velocities[1], msg.velocities[0], msg.velocities[2];
    planned_a << -msg.accelerations[1], msg.accelerations[0], msg.accelerations[2];

//    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];
//    planned_yaw = msg->positions[3];
//    planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
//    planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];


    /// Publish to record in rosbag
    nav_msgs::Odometry odom_sp_enu;
    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = planned_p(0);
    odom_sp_enu.pose.pose.position.y = planned_p(1);
    odom_sp_enu.pose.pose.position.z = planned_p(2);
    odom_sp_enu.twist.twist.linear.x = planned_v(0);
    odom_sp_enu.twist.twist.linear.y = planned_v(1);
    odom_sp_enu.twist.twist.linear.z = planned_v(2);
    odom_sp_enu_pub.publish(odom_sp_enu);

    static Vector3d p_error_last;
    static Vector3d v_error_last;
    static Vector3d p_error_accumulate;
    static Vector3d v_error_accumulate;
    static bool if_init = true;


    /// Calculate desired thrust and attitude
    Vector3d p_error = planned_p - current_p;
    Vector3d delt_p_error = p_error - p_error_last;

    planned_v += vectorElementMultiply(p_error, position_error_p) + vectorElementMultiply(delt_p_error, position_error_d) + vectorElementMultiply(p_error_accumulate, position_error_i);
    Vector3d v_error = planned_v - current_v;

    if(if_init){
        if_init = false;
        p_error_last = p_error;
        v_error_last = v_error;
        p_error_accumulate = p_error;
        v_error_accumulate = v_error;
        return;
    }

    Vector3d delt_v_error = v_error - v_error_last;

    p_error_accumulate += p_error;
    v_error_accumulate += v_error;
    vector3dLimit(p_error_accumulate, p_i_acc_error_limit);
    vector3dLimit(v_error_accumulate, v_i_acc_error_limit);

    Vector3d a_fb =   /// PID
            vectorElementMultiply(v_error, velocity_error_p) +
            vectorElementMultiply(delt_v_error, velocity_error_d) +
            vectorElementMultiply(v_error_accumulate, velocity_error_i) +
            vectorElementMultiply(p_error, position_error_ff);

    // Set a maximum acceleration feedforward value given by position and velocity error.
    for(int i=0; i<3; i++){
        if(fabs(a_fb(i)) > 5.0) a_fb(i) = 5.0 * a_fb(i) / fabs(a_fb(i));
    }

    p_error_last = p_error;
    v_error_last = v_error;

    Vector3d z_w_norm(0, 0, 1.0);
    Vector3d a_des = a_fb + planned_a + GRAVITATIONAL_ACC * z_w_norm;

    Eigen::Vector3d expected_thrust = a_des;
    double expected_yaw = planned_yaw;

    Eigen::Vector3d body_x, body_y, body_z;
    if (expected_thrust.norm() > 0.00001f) {
        body_z = expected_thrust / expected_thrust.norm();  //Normalize
    } else {
        // no thrust, set Z axis to safe value
        body_z << 0.f, 0.f, 1.f;
    }

    // vector of desired yaw direction in XY plane rotated by PI/2
    Eigen::Vector3d y_C(-sin(expected_yaw), cos(expected_yaw), 0.0f); // ???? chg

    if (fabs(body_z(2)) > 0.000001) {
        // desired body_x axis, orthogonal to body_z
        body_x = y_C.cross(body_z);

        // keep nose to front while inverted upside down
        if (body_z(2) < 0.0f) {
            body_x = -body_x;
        }

        body_x = body_x;

    } else {
        // desired thrust is in XY plane, set X downside to construct correct matrix,
        // but yaw component will not be used actually
        body_x = Eigen::Vector3d::Zero();
        body_x(2) = 1.0f;
    }

    body_y = body_z.cross(body_x);

    Eigen::Matrix3d rotation_matrix;
    for (int i = 0; i < 3; i++) {
        rotation_matrix(i, 0) = body_x(i);
        rotation_matrix(i, 1) = body_y(i);
        rotation_matrix(i, 2) = body_z(i);
    }

    Quaterniond q_target;
    q_target = rotation_matrix;


    //Calculate thrust
    double thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

    /**End of Core code**/

    att_setpoint.header.stamp = ros::Time::now();
    att_setpoint.orientation.w = q_target.w();
    att_setpoint.orientation.x = q_target.x();
    att_setpoint.orientation.y = q_target.y();
    att_setpoint.orientation.z = q_target.z();
    att_setpoint.thrust = thrust_des;


    ROS_INFO_THROTTLE(1.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
                      att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);

    att_ctrl_pub.publish(att_setpoint);
}




void pvaCallback(const trajectory_msgs::JointTrajectoryPoint& msg)
{
    ROS_WARN("Commands received.");

    pva_command_received = true;
    pvaPointSet(msg);
}



void configureCallback(tracker::PVA_TrackerConfig &config, uint32_t level) {
    position_error_p << config.position_p_xy, config.position_p_xy, config.position_p_z;
    position_error_ff << config.position_ff_xy, config.position_ff_xy, 0.0;
    position_error_d << config.position_d_xy, config.position_d_xy, config.position_d_z;
    position_error_i << config.position_i_xy, config.position_i_xy, config.position_i_z;
    p_i_acc_error_limit = config.p_i_acc_error_limit;

    velocity_error_p << config.velocity_p_xy, config.velocity_p_xy, config.velocity_p_z;
    velocity_error_d << config.velocity_d_xy, config.velocity_d_xy, config.velocity_d_z;
    velocity_error_i << config.velocity_i_xy, config.velocity_i_xy, config.velocity_i_z;
    v_i_acc_error_limit = config.v_i_acc_error_limit;
    thrust_factor = config.hover_thrust_factor_min; //initial value. Consider the battery is fully charged.
    thrust_factor_max = config.hover_thrust_factor_max;
    thrust_factor_min = config.hover_thrust_factor_min;
    max_flight_time = config.flight_time_minute;
    max_stand_by_time = config.stand_by_time_minute;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker");

    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig> server;
    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig>::CallbackType f;
    f = boost::bind(&configureCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);

    ros::Subscriber odom_sub = nh.subscribe("/Bebop2/position_velocity_orientation_estimation", 1, odomCallback);

    att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    
    pose_estimation_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    
    odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_enu", 1);

    ros::Rate loop_rate(20);
    trajectory_msgs::JointTrajectoryPoint take_off_hover_pose;
    take_off_hover_pose.positions.push_back(0);
    take_off_hover_pose.positions.push_back(0);
    take_off_hover_pose.positions.push_back(0);
    take_off_hover_pose.positions.push_back(0);

    take_off_hover_pose.velocities.push_back(0);
    take_off_hover_pose.velocities.push_back(0);
    take_off_hover_pose.velocities.push_back(0);

    take_off_hover_pose.accelerations.push_back(0);
    take_off_hover_pose.accelerations.push_back(0);
    take_off_hover_pose.accelerations.push_back(0);


    // wait for FCU connection and update current position
    int delay_counter = 0;
    double takeoff_height = 1.0;

    while(ros::ok() && (!current_state.connected || delay_counter < 60)){
        delay_counter++;

        take_off_hover_pose.positions[0] = current_p(0);
        take_off_hover_pose.positions[1] = current_p(1);
        take_off_hover_pose.positions[2] = takeoff_height;

        ros::spinOnce();
        loop_rate.sleep();
    }


    /***************** Arm and switch to offboard mode, only use this in simulation tests *********************************/
//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//            ("mavros/cmd/arming");
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//            ("mavros/set_mode");
//
//
//    mavros_msgs::SetMode offb_set_mode;
//    offb_set_mode.request.custom_mode = "OFFBOARD";
//
//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;
//    ros::Time last_request = ros::Time::now();
//
//    ROS_INFO("Arm and takeoff");
//    while(ros::ok()){
//        if( current_state.mode != "OFFBOARD" &&
//            (ros::Time::now() - last_request > ros::Duration(5.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("Offboard enabled");
//            }
//            last_request = ros::Time::now();
//        } else {
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(5.0))){
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                }
//                last_request = ros::Time::now();
//            }
//        }
//
//        trajectory_msgs::JointTrajectoryPoint pva_setpoint;
//
//
//
//        if(current_state.mode != "OFFBOARD" || !current_state.armed){
//            pvaPointSet(take_off_hover_pose);
//        }else{
//            break;
//        }
//
//        loop_rate.sleep();
//        ros::spinOnce();
//    }
//
//    ROS_WARN("ARMED in OFFBOARD mode. Waiting for commands.");
    /********************************************************************************************************************/



    while(ros::ok() && (!pva_command_received || current_state.mode != "OFFBOARD" )){

        pvaPointSet(take_off_hover_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
