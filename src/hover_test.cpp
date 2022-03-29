/*********************************
 * Created by cc on 2020/8/5.
 * This file generates hovering commands
 ********************************/


#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
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



int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);


    /// Take off with constant acceleration
    double yaw_set = 0.0;
    if (nh.getParam("/hover_test/yaw_set", yaw_set)) {
        ROS_INFO("get param hover_height: %f", yaw_set);
    }

    Vector3d recorded_hover_position(current_p(0), current_p(1), current_p(2));
    Vector3d v_set(0.0, 0.0, 0.0);
    Vector3d a_set(0.0, 0.0, 0.0);


    ros::Rate loop_rate(20);
    /** Wait for offboard **/
    while(ros::ok()){
        setPVA(recorded_hover_position, v_set, a_set, yaw_set);//a_t.row(last_index));

        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            recorded_hover_position << current_p(0), current_p(1), current_p(2);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
