
#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <istream>
#include <time.h>

#include <geometry_msgs/Twist.h> // For geometry_msgs:: Twist
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <geometry_msgs/PoseArray.h>

#include "mono_pub/Position.h"
#include "mono_pub/Attitude.h"
#include "mono_pub/Num.h"
#include "mono_pub/MonoMsg.h"
#include "guidance/distance.h"

#define V_MAX 1   
#define VZ_MAX 1.5 
#define VA_MAX 10.0 //deg
#define KX 0.25
#define KY 0.25
#define KZ 0.4
#define KA 0.5
#define pi 3.14159265359
#define R2D 57.29578
#define D2R 0.0174533
#define SprayDistance 0.8

#define Higher2Figure 0.2
#define Distance2Figure 1.2
#define VtoWall 0.15

#define FigureCloseEnough 0.2

#define DeltaX 0.2
#define DeltaY 0.2
#define DeltaZ 0.15

using namespace std;
using namespace DJI::onboardSDK;
using namespace Eigen;

// dji_sdk::LocalPosition local;
// dji_sdk::AttitudeQuaternion q;
// std_msgs::UInt8 myflight_status;

// double yaw_R;
// double yaw0_D;
// double yaw0_R;
// float32_t X, Y, yawT_R;
// 
// // target number position  10 numbers in total
// double TargetNumber_Position[10][4];  // 0 - 10
// double Screen_Position[3];
// double Screen_Yaw;
// 
// double Height;
// double DistanceToWall;
// 
// double yaw;
// double pitch;
// double roll;
// 
// double VtoWall;
// 
// double Target_X;
// double Target_Y;
// double Target_Z;
// 
// double StartTime;
// double MissionDuration;

// call back
void ultrasonic_callback(sensor_msgs::LaserScan g_ul);
void local_position_callback(const dji_sdk::LocalPosition::ConstPtr& msg);
void AttitudeQuaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr& q);
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
void messageCallback(const mono_pub::MonoMsg::ConstPtr& msg);
void landingCallback(const guidance::distance::ConstPtr& msg);

//void imu_callback(const geometry_msgs::TransformStamped g_imu);
//void position_callback(const geometry_msgs::Vector3Stamped& g_pos);

// function
int Takeoff(DJIDrone* drone);
void Initialization(DJIDrone* drone, ros::NodeHandle n);
int TargetFigureRecognization(DJIDrone* drone, ros::NodeHandle n);
void SprayMission(DJIDrone* drone, ros::NodeHandle n, int TargetFigure);
void Landing(ros::NodeHandle n, DJIDrone* drone);

void FlyToTargetPoint(DJIDrone* drone,ros::NodeHandle a,float inputx,float inputy,float inputz,float yawT_D);
void MoveAtCertainVelocity(DJIDrone* drone,ros::NodeHandle a,float inputvx,float inputvy,float inputvz,float inputvyaw);

int Timer();
void Quaternion_To_Euler(float q0,float q1,float q2,float q3);
void ParameterInput();

#endif


