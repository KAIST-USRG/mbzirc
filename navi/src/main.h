#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
//#include "navi/rio_to_pc.h"
#include "navi/test.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include "common_def.h"
#include "CoordinateConv.h"
#include "c_ekf.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #define M_PI 3.1415926535897932384626433832795

using namespace std;

CoordinateConv m_CoordConv;
c_ekf m_ekf;

double m_d_nav_result[12];

//////////////////////////////////////////////////////////
struct timespec prev_time;
struct timespec curr_time;
double interval = 0.0;

IMU_data m_imu_data;
IMU_data m_imu_data_prev;
GPS_data m_gps_data;
GPS_data m_gps_data_prev;
ODOM_data m_odom_data;
ODOM_data m_odom_data_prev;

double m_dRef_UTM_X = 0.0;
double m_dRef_UTM_Y = 0.0;
double m_dGPS_UTM_X = 0.0;
double m_dGPS_UTM_Y = 0.0;
double m_dGPS_UTM_Heading = 0.0;
double m_dGPS_UTM_Heading_prev = 0.0;
bool heading_update = true;
bool pose_update = true;


double m_pos_x_local = 0.0;
double m_pos_y_local = 0.0;
double m_gps_x_local = 0.0;
double m_gps_y_local = 0.0;

bool GPS_init = false;
double initial_heading = 0.0;

double imu_yaw_rate = 0.0;
double vehicle_speed = 0.0;

bool first_call = true;
double gps_ref[3][2] = {
    { 37.243071, 126.774043 }, // K-CITY
    { 36.368180, 127.363873 }, // KAIST KI building
    { 36.392309, 127.399654 }, // KAIST Munji campus
};
// distance comparison between current point and gps_ref
double distanceTemp[3] = {0,};
double minimum_value = 0.0;
int ref_index = 0;
Mat vel_and_yawRate, GPS_data, mat_init_with_GPS;

// 0~3 km/h is noisy at TRAM. So, threshold is 3.0 km/h
double update_threshold_velocity = 0.4;
double heading_update_threshold_speed_x = 1.0;
double heading_update_threshold_speed_y = 0.1;
double heading_update_threshold_angular_z = 0.01;

double count_test = 0.0;

double gps_count_prev = 0.0;
double gps_count_curr = 0.0;
bool gps_flag = false;
double roll;
double pitch;
double yaw;
double vel_y;

bool init_with_GPS = true;

geometry_msgs::Twist twist_msg;
