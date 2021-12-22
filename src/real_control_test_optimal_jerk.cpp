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

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    current_p << msg->pose.position.y, -msg->pose.position.x, msg->pose.position.z;
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

//    ROS_INFO("P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO("V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO("A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
}

/** This function is to generate state to state trajectory **/
void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, double v_max, double delt_t,
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    // % Choose the time as running in average velocity
    // double decay_parameter = 0.5;
    // double T = 0.2;

    double j_limit = 4;
    double a_limit = 2;
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
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);


    const int LOOPRATE = 40;
    ros::Rate loop_rate(LOOPRATE);

    /// Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    double delt_t = 1.0 / LOOPRATE;
    double yaw_target = M_PI / 2.0;

    Eigen::MatrixXd p, v, a;
    Eigen::VectorXd t;
    Eigen::Vector3d pf;
    pf << 1.5, 0, 0;
    motion_primitives(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), pf, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                      1.0, delt_t, p, v, a, t);
    int one_side_size = p.rows();

    Vector3d start_p0 = current_p;
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            setPVA(current_p, Vector3d::Zero(), Vector3d::Zero(), yaw_target);
            start_p0 = current_p;
        }else{

            /** Hover for 10 seconds **/
            for(int i=0; i<400; i++){
                setPVA(start_p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw_target);

                loop_rate.sleep();
                ros::spinOnce();
            }

            /** Now fly a square path with minimum jerk planned path **/
            int side = 0;
            int counter = 0;
            Eigen::Vector3d p_last;

            while(ros::ok()){
                Eigen::Vector3d p_sp, v_sp, a_sp;

                if(counter >= one_side_size){
                    side += 1;
                    if(side > 3) side = 0;
                    start_p0 = p_last;
                    counter = 0;
                }

                switch(side){
                case 0:  // +x direction
                    p_sp = start_p0;
                    p_sp(0) += p(counter, 0);
                    v_sp = Eigen::Vector3d::Zero();
                    v_sp(0) = v(counter, 0);
                    a_sp = Eigen::Vector3d::Zero();
                    a_sp(0) = a(counter, 0);
                    counter ++;
                    break;
                case 1:  //+y direction
                    p_sp = start_p0;
                    p_sp(1) += p(counter, 0);
                    v_sp = Eigen::Vector3d::Zero();
                    v_sp(1) = v(counter, 0);
                    a_sp = Eigen::Vector3d::Zero();
                    a_sp(1) = a(counter, 0);
                    counter ++;
                    break;
                case 2:  //-x direction
                    p_sp = start_p0;
                    p_sp(0) -= p(counter, 0);
                    v_sp = Eigen::Vector3d::Zero();
                    v_sp(0) = -v(counter, 0);
                    a_sp = Eigen::Vector3d::Zero();
                    a_sp(0) = -a(counter, 0);
                    counter ++;
                    break;
                case 3:  //-y direction
                    p_sp = start_p0;
                    p_sp(1) -= p(counter, 0);
                    v_sp = Eigen::Vector3d::Zero();
                    v_sp(1) = -v(counter, 0);
                    a_sp = Eigen::Vector3d::Zero();
                    a_sp(1) = -a(counter, 0);
                    counter ++;
                    break;
                }

                setPVA(p_sp, v_sp, a_sp, yaw_target);//a_t.row(last_index));
                p_last = p_sp;

                if(current_state.mode != "OFFBOARD" || !current_state.armed){
                    break;
                }

                loop_rate.sleep();
                ros::spinOnce();
            }

        }
        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
