#include "main.h"
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

Euler Quarter2Eular(Quaternion _q)
{
    Euler out;

    float sinr = +2.0 * (_q.w * _q.x + _q.y * _q.z);
    float cosr = +1.0 -2.0 * (_q.x * _q.x + _q.y * _q.y);

    out.roll = atan2(sinr,cosr);

    float sinp = +2.0 * (_q.w * _q.y - _q.z * _q.x);

    if(fabs(sinp) >=1)
    {
        out.pitch = copysign(M_PI / 2, sinp);
    }
    else
    {
        out.pitch = asin(sinp);
    }

    float siny = 2.0 * (_q.w * _q.z + _q.x * _q.y);
    float cosy = 1.0 -2.0 * (_q.y * _q.y + _q.z * _q.z);
    out.yaw = atan2(siny,cosy);

    return out;
}


void subscribeCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    m_imu_data_prev = m_imu_data;

    m_imu_data.angular_vel_x = msg->angular_velocity.x;
    m_imu_data.angular_vel_y = msg->angular_velocity.y;
    m_imu_data.angular_vel_z = msg->angular_velocity.z;

    m_imu_data.linear_acceleration_x = msg->linear_acceleration.x;
    m_imu_data.linear_acceleration_y = msg->linear_acceleration.y;
    m_imu_data.linear_acceleration_z = msg->linear_acceleration.z;

    // swap X/Y, invert Z
    // microstrain 3dm-gx5 package is modified by author
    m_imu_data.orientation_w = msg->orientation.w;
    m_imu_data.orientation_x = msg->orientation.y;
    m_imu_data.orientation_y = msg->orientation.x;
    m_imu_data.orientation_z = -1.0*msg->orientation.z;

    // swap X/Y, invert Z
    // microstrain 3dm-gx5 package is modified by author
    Quaternion tmp;
    tmp.w = msg->orientation.w;
    tmp.x = msg->orientation.y;
    tmp.y = msg->orientation.x;
    tmp.z = -1.0*msg->orientation.z;

    Euler e = Quarter2Eular(tmp);

    m_imu_data.euler_angle_x = e.roll*180.0/M_PI;
    m_imu_data.euler_angle_y = e.pitch*180.0/M_PI;
    m_imu_data.euler_angle_z = e.yaw*180.0/M_PI;

    count_test++;

}

void subscribeCallback_gps(const std_msgs::Float64MultiArray& msg)
{
    m_gps_data_prev = m_gps_data;

    int time_hour = 0;
    int time_minute = 0;
    double time_second = 0.0;
    int time_ms = 0;

    time_hour = msg.data.at(0);
    time_minute = msg.data.at(1);
    time_second = msg.data.at(2);

    time_ms = time_hour*3600000 + time_minute*60000 + time_second*1000;

    int rmc_time_hour = 0;
    int rmc_time_minute = 0;
    double rmc_time_second = 0.0;
    int rmc_time_ms = 0;

    rmc_time_hour = msg.data.at(12);
    rmc_time_minute = msg.data.at(13);
    rmc_time_second = msg.data.at(14);

    rmc_time_ms = rmc_time_hour*3600000 + rmc_time_minute*60000 + rmc_time_second*1000;

    double gps_height1 = 0.0;
    double gps_height2 = 0.0;

    gps_height1 = msg.data.at(10); // height (at virtual sea level)
    gps_height2 = msg.data.at(11); // height (difference between real sea level and virtual sea level)

    double dGPS_DMLAT = 0.0;
    double dGPS_DMLONG = 0.0;
    dGPS_DMLAT = msg.data.at(3); // latitude
    dGPS_DMLONG = msg.data.at(5); // longitude

    // DegMin -> DegDeg
    m_CoordConv.GPSWGS84_DM2DD(dGPS_DMLAT, dGPS_DMLONG);

    m_gps_data.time = time_ms; // gps time
    m_gps_data.latitude = m_CoordConv.dWGS84_DDLAT;
    m_gps_data.lat_direction = msg.data.at(4); // latitude direction
    m_gps_data.longitude = m_CoordConv.dWGS84_DDLON;
    m_gps_data.long_direction = msg.data.at(6); // longitude direction
    m_gps_data.quality_indicator = 1; // mode indicator
    m_gps_data.num_use_satellite = msg.data.at(8); // # of using satellites
    m_gps_data.HDOP = msg.data.at(9); // Horizontal dillusion of position
    m_gps_data.height = gps_height1 + gps_height2;

    m_gps_data.rmc_time = rmc_time_ms;
    m_gps_data.gprmc_speed = msg.data.at(20); // GxRMC knots to m/s speed
    m_gps_data.gprmc_gps_heading = m_dGPS_UTM_Heading; // GxRMC heading degree (north base clock-wise direction 0~360 deg)

    m_CoordConv.WGS2UTM(gps_ref[ref_index][0], gps_ref[ref_index][1]);
    m_dRef_UTM_X = m_CoordConv.dUTM_X;
    m_dRef_UTM_Y = m_CoordConv.dUTM_Y;
    m_CoordConv.WGS2UTM(m_gps_data.latitude, m_gps_data.longitude);
    m_dGPS_UTM_X = m_CoordConv.dUTM_X;
    m_dGPS_UTM_Y = m_CoordConv.dUTM_Y;

    m_gps_data.pos_x = m_dGPS_UTM_X - m_dRef_UTM_X;
    m_gps_data.pos_y = m_dGPS_UTM_Y - m_dRef_UTM_Y;

}

void subscribeCallback_utm(const nav_msgs::OdometryConstPtr& msg)
{
    gps_count_prev = gps_count_curr;
    //data subscribe
    m_dGPS_UTM_X = msg->pose.pose.position.x;
    m_dGPS_UTM_Y = msg->pose.pose.position.y;

    m_gps_data.pos_x = m_dGPS_UTM_X ;
    m_gps_data.pos_y = m_dGPS_UTM_Y ;
    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion quat_init_utm;
    quat_msg = msg->pose.pose.orientation;
    tf2::convert(quat_msg, quat_init_utm);
    tf2::Matrix3x3 mat(quat_init_utm);
    mat.getRPY(roll, pitch, yaw);
    m_dGPS_UTM_Heading_prev = m_dGPS_UTM_Heading;
    m_dGPS_UTM_Heading = yaw * 180 / M_PI;
    double heading_update_thres_speed = 1.2;


    if ( sqrt(m_odom_data.velocity * m_odom_data.velocity + vel_y * vel_y) > update_threshold_velocity){
        pose_update = true;
        if (fabs(m_odom_data.velocity) > heading_update_thres_speed && fabs(m_imu_data.angular_vel_z) < 0.1){
            heading_update = true;    
        }
        else{
            heading_update = false;
        }
    }
    else{
        pose_update = false;
    }  



    gps_count_curr = gps_count_curr + 1;

    if(gps_count_curr != gps_count_prev){
        gps_flag = true;
    }

    if(gps_count_curr == 2000000000){
        gps_count_curr = gps_count_prev = 0;
    }
}


void subscribeCallback_nav_odom(const nav_msgs::Odometry& msg)
{
    m_odom_data_prev = m_odom_data;
    twist_msg = msg.twist.twist;

    m_odom_data.velocity = msg.twist.twist.linear.x; //vehicle velocity
    vel_y = msg.twist.twist.linear.y;
    // std::cout << m_odom_data.velocity << std::endl;
}

void subscribeCallback_init(const std_msgs::String::ConstPtr& msg)
{
    GPS_init = true;
}
int main(int argc, char** argv)
{
    interval = 0.05;

    ros::init(argc, argv, "navi");
    ros::NodeHandle nh;

    // ros::Publisher pub_pose_estimation = nh.advertise<std_msgs::Float64MultiArray>("POS_T",1);
    ros::Publisher pub_ekf_odom_estimation = nh.advertise<nav_msgs::Odometry>("/odometry/EKF_estimated",1);
    ros::Subscriber listener_imu = nh.subscribe("/gx5/imu/data",1, subscribeCallback_imu, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_odom = nh.subscribe("/odom",1, subscribeCallback_nav_odom, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_init = nh.subscribe("init",100, subscribeCallback_init, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_utm = nh.subscribe("/outdoor_nav/odometry/gps_utm", 100,&subscribeCallback_utm,ros::TransportHints().tcpNoDelay());

    std_msgs::Float64MultiArray msg_ekf_pose_result_publish;
    nav_msgs::Odometry ekf_odom_msg;
    geometry_msgs::Quaternion odom_quat;

    ros::Rate loop_rate(20);

    bool cmp_gps_time = false;
    bool cmp_gps_rmc_time = false;
    bool cmp_num_setellite_and_dev = false;
    int countpos = 0;

    bool get_position = false;
    bool get_heading = false;
    bool POS_init = true;

    double hcount = 0.0;
    double hbias = 0.0;
    double h_gps = 0.0;

    double vcount = 0.0;
    double v_gain = 1.0;
    double v_gain_temp = 0.0;

    double heading_error = 0.0;
    double pos_error = 0.0;

    initial_heading = -29.545*M_PI/180.0;
    h_gps = initial_heading;

    double vel_ave = 0.0;
    double c_yaw_rate = 0.0;

    vel_and_yawRate = Mat::zeros(3, 1, CV_64FC1);
    GPS_data = Mat::zeros(3, 1, CV_64FC1);

    bool true_false_1st = false;
    bool true_false_2nd = false;
    bool ekf_update_bool = false;
    bool pos_update_boolean = false;

    for(int n =0; n<12; n++){
        m_d_nav_result[n] = 0.0;
    }

    m_ekf.EKF_Initialization();

    while (ros::ok()) {
        // interval time starting point
        clock_gettime(CLOCK_MONOTONIC, &prev_time);

        msg_ekf_pose_result_publish.data.clear();
        ros::spinOnce();

        imu_yaw_rate = m_imu_data.angular_vel_z;
        vehicle_speed = m_odom_data.velocity;
        // ROS_INFO("IMU: %f, Speed: %f, Update_thres: %f",imu_yaw_rate, vehicle_speed, update_threshold_velocity);
        
        if(vehicle_speed > update_threshold_velocity){
            // h_gps = (-1.0*m_dGPS_UTM_Heading+90.0)*M_PI/180.0;
            h_gps = (m_dGPS_UTM_Heading)*M_PI/180.0;
            h_gps = m_ekf.AngDiff(h_gps);
        }
        if(vehicle_speed < (-1.0*update_threshold_velocity)){ // backward threshold is -3.0 km/h
            // h_gps = (-1.0*m_dGPS_UTM_Heading+90.0)*M_PI/180.0 - M_PI;
            h_gps = (m_dGPS_UTM_Heading)*M_PI/180.0 - M_PI;
            h_gps = m_ekf.AngDiff(h_gps);
        }
        /////////////////////////////////////////////////////

        ////////////// heading bias calculate ///////////////
        if (fabs(vehicle_speed) <= update_threshold_velocity){
            hcount += 1.0;
            if(hcount > 18446744073709551615.0){ hcount = 18446744073709551615.0; }
            hbias = (hbias*hcount + imu_yaw_rate)/hcount;
        }
        /////////////////////////////////////////////////////
        // ROS_INFO("%f", hbias);
        vel_ave = vehicle_speed;
        // if( fabs(vehicle_speed) <= update_threshold_velocity){
        //     c_yaw_rate = 0.0;
        // }
        // else {
        //     // c_yaw_rate = (imu_yaw_rate - hbias)*(-1.0); // this case : right turn is plus
        //     // c_yaw_rate = (imu_yaw_rate - hbias); // this case : left turn is plus
        //     c_yaw_rate = (imu_yaw_rate);
        // }
        c_yaw_rate = (imu_yaw_rate);
        vel_and_yawRate.ptr<double>(0)[0] = m_odom_data.velocity; //vel_x
        vel_and_yawRate.ptr<double>(1)[0] = vel_y;   //vel_y
        vel_and_yawRate.ptr<double>(2)[0] = c_yaw_rate;      
        GPS_data.ptr<double>(0)[0] = m_gps_data.pos_x;
        GPS_data.ptr<double>(1)[0] = m_gps_data.pos_y;
        GPS_data.ptr<double>(2)[0] = h_gps;

        m_ekf.EKF_Predictionstep(m_ekf.m_xhat, m_ekf.m_Phat, vel_and_yawRate, interval);
        // cout<<"pose_update"<< pose_update <<endl;
        // if (gps_flag == true){
        //     if (pose_update ==true){
        //         if (heading_update == true){
        //             m_ekf.EKF_Correctionstep(m_ekf.m_xhat, m_ekf.m_Phat, m_gps_data.quality_indicator, true, true, GPS_data); //check
        //         }
        //         else{
        //             m_ekf.EKF_Correctionstep(m_ekf.m_xhat, m_ekf.m_Phat, m_gps_data.quality_indicator, true, false, GPS_data); //check
        //         }
        //     }
        //     else{
        //         m_ekf.EKF_Correctionstep(m_ekf.m_xhat, m_ekf.m_Phat, m_gps_data.quality_indicator, false, false, GPS_data);            
        //     }
        // }

        gps_flag = false;

        ekf_odom_msg.header.stamp = ros::Time::now();
        ekf_odom_msg.header.frame_id = "odom";
        ekf_odom_msg.child_frame_id = "base_footprint";
        ekf_odom_msg.pose.pose.position.x = m_ekf.m_xhat.ptr<double>(0)[0];
        ekf_odom_msg.pose.pose.position.y = m_ekf.m_xhat.ptr<double>(1)[0];
        tf2::Quaternion quat_ekf;
        geometry_msgs::Quaternion quat_ekf_msg;
        double angle_z_rad = m_ekf.m_xhat.ptr<double>(2)[0];
        if (std::isnan(angle_z_rad)){
            angle_z_rad = 0;
        }
        quat_ekf.setRPY(0,0,angle_z_rad);
        quat_ekf.normalize();
        quat_ekf_msg = tf2::toMsg(quat_ekf);

        ekf_odom_msg.pose.pose.orientation= quat_ekf_msg;
        ekf_odom_msg.twist.twist = twist_msg;
        pub_ekf_odom_estimation.publish(ekf_odom_msg);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(m_ekf.m_xhat.ptr<double>(0)[0], m_ekf.m_xhat.ptr<double>(1)[0], 0) ); 
        transform.setRotation(tf::Quaternion(0, 0, angle_z_rad));
        br.sendTransform(tf::StampedTransform(transform, ekf_odom_msg.header.stamp, "odom", "base_footprint"));


        // static tf::TransformBroadcaster br_;
        // tf::Transform transform_;
        // transform_.setOrigin( tf::Vector3(m_ekf.m_xhat.ptr<double>(0)[0], m_ekf.m_xhat.ptr<double>(1)[0], 0) ); 
        // transform_.setRotation(tf::Quaternion(0, 0, angle_z_rad));
        // br_.sendTransform(tf::StampedTransform(transform_, ekf_odom_msg.header.stamp, "odom", "velodyne"));


        m_imu_data_prev = m_imu_data;
        m_odom_data_prev = m_odom_data;
        m_gps_data_prev = m_gps_data;

        loop_rate.sleep();

        // calculate interval time
        clock_gettime(CLOCK_MONOTONIC, &curr_time);
        interval = (curr_time.tv_nsec - prev_time.tv_nsec)/1000000000.0; /* seconds */
        // if (!(interval > 0.0098 && interval < 0.012)){
        //     interval = 0.01;
        // }
        // cout << "interval: " << interval << endl;
        // cout << m_d_nav_result[2] <<endl;
    }

    return 0;
}
