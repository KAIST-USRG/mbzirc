#include "c_ekf.h"
#include <cmath>

c_ekf::c_ekf()
{
    EKF_Initialization();
}

c_ekf::~c_ekf()
{
    m_xhat.release();
    m_Phat.release();
    m_Q.release();
    m_R1.release();
    m_R2.release();
    m_R3.release();
}

void c_ekf::EKF_Initialization()
{
    m_xhat = Mat::zeros(3, 1, CV_64FC1);
    m_Phat = Mat::zeros(3, 3, CV_64FC1);

    m_Q = Mat::zeros(3, 3, CV_64FC1);

    // x, y, velocity, heading, yaw_rate
    m_Q.ptr<double>(0)[0] = 0.5;
    m_Q.ptr<double>(1)[1] = 0.5;
    m_Q.ptr<double>(2)[2] = 0.0341;

    m_R1 = Mat::zeros(3, 3, CV_64FC1);
    m_R2 = Mat::zeros(3, 3, CV_64FC1);
    m_R3 = Mat::zeros(3, 3, CV_64FC1);

    // gps single
    m_R1.ptr<double>(0)[0] = 20;//10;
    m_R1.ptr<double>(1)[1] = 20;//10;
    // m_R1.ptr<double>(2)[2] = 0.5;
    m_R1.ptr<double>(2)[2] = 4;//2;

    // gps rtk-float
    m_R2.ptr<double>(0)[0] = 2;
    m_R2.ptr<double>(1)[1] = 2;
    m_R2.ptr<double>(2)[2] = 0.5;

    // gps rtk-fixed
    m_R3.ptr<double>(0)[0] = 0.5;
    m_R3.ptr<double>(1)[1] = 0.5;
    m_R3.ptr<double>(2)[2] = 0.5;
}

void c_ekf::EKF_Predictionstep(Mat xhat, Mat Phat, Mat odom, double dt)
{
    double pos_x, pos_y, vel, heading, yaw_rate;
    double pos_x_prev, pos_y_prev, heading_prev;
    double vel_x, vel_y, o_x, o_y, p_x, p_y, dot, det, mag, vel_angle_rad;
    o_x = 1;  //local axis reference (1,0) vector
    o_y = 0;  //local axis reference (1,0) vector
    vel_x = odom.ptr<double>(0)[0];
    vel_y = odom.ptr<double>(1)[0];
    yaw_rate = odom.ptr<double>(2)[0];
    p_x = vel_x*dt;
    p_y = vel_y*dt;
    dot = o_x*p_x + o_y*p_y;
    det = o_x*p_y - o_y*p_x;
    if (dot==0){
        dot = 0.0000001;
    }
    vel_angle_rad = atan2(det,dot);
    mag = sqrt(pow(p_x,2)+pow(p_y,2));
    pos_x_prev = xhat.ptr<double>(0)[0];
    pos_y_prev = xhat.ptr<double>(1)[0];
    heading_prev = xhat.ptr<double>(2)[0];
    heading = heading_prev + yaw_rate*dt;
    // cout << "yaw_estimated: " << heading << endl;

    heading = AngDiff(heading);
    // cout << "dt: " << dt << endl;
    // cout << "yaw_rate: " << yaw_rate << endl;
    // cout.precision(8);    
    // cout << "yaw_estimated: " << heading << endl;
    // cout << "move_heading: " << heading+vel_angle_rad << endl;
    pos_x = pos_x_prev + mag*cos(heading+vel_angle_rad);
    pos_y = pos_y_prev + mag*sin(heading+vel_angle_rad);
    
    xhat.ptr<double>(0)[0] = pos_x;
    xhat.ptr<double>(1)[0] = pos_y;
    xhat.ptr<double>(2)[0] = heading;

    Mat A;
    A = Mat::zeros(3, 3, CV_64FC1);

    A.ptr<double>(0)[0] = 1;  A.ptr<double>(0)[1] = 0; A.ptr<double>(0)[2] = mag*(cos(vel_angle_rad)*-sin(heading)-sin(vel_angle_rad)*cos(heading));
    A.ptr<double>(1)[0] = 0;  A.ptr<double>(1)[1] = 1; A.ptr<double>(1)[2] = mag*(cos(heading)*cos(vel_angle_rad)-sin(heading)*sin(vel_angle_rad));
    A.ptr<double>(2)[0] = 0;  A.ptr<double>(2)[1] = 0; A.ptr<double>(2)[2] = 1;

    // A.t() : Transpose
    Phat = A*Phat*A.t() + m_Q;

    m_Phat = Phat.clone();
    m_xhat = xhat.clone();
}

void c_ekf::EKF_Correctionstep(Mat xhat, Mat Phat, double gps_qual_indicator, bool pos_flag, bool heading_flag, Mat zMeasureSensor)
{
    Mat zhat, I, H, StInverse;

    zhat = Mat::zeros(3, 1, CV_64FC1);
    zhat.ptr<double>(0)[0] = xhat.ptr<double>(0)[0];
    zhat.ptr<double>(1)[0] = xhat.ptr<double>(1)[0];    
    zhat.ptr<double>(2)[0] = xhat.ptr<double>(2)[0];

    I = Mat::eye(3, 3, CV_64FC1);

    H = Mat::zeros(3, 3, CV_64FC1);

    H.ptr<double>(0)[0] = 1.0;
    H.ptr<double>(1)[1] = 1.0;
    H.ptr<double>(2)[2] = 1.0;

    if(gps_qual_indicator == 4){
        StInverse = H*Phat*H.t() + m_R3;
    }
    else if(gps_qual_indicator == 5){
        StInverse = H*Phat*H.t() + m_R2;
    }
    else {
        StInverse = H*Phat*H.t() + m_R1;
    }
    K = Phat*H.t()*StInverse.inv();
    if(pos_flag == false){
        K.ptr<double>(0)[0] = 0.0;
        K.ptr<double>(1)[1] = 0.0;
    }
    if(heading_flag == false){
        K.ptr<double>(2)[2] = 0.0;
    }
    if(fabs(zhat.ptr<double>(2)[0] - zMeasureSensor.ptr<double>(2)[0]) > M_PI){
        if(zhat.ptr<double>(2)[0] - zMeasureSensor.ptr<double>(2)[0] > M_PI){
            zMeasureSensor.ptr<double>(2)[0] = zMeasureSensor.ptr<double>(2)[0] + 2.0*M_PI;
        }
        else {
            zMeasureSensor.ptr<double>(2)[0] = zMeasureSensor.ptr<double>(2)[0] - 2.0*M_PI;
        }
    }
    xhat = xhat + K*(zMeasureSensor - zhat);
    Phat = (I-K*H)*Phat;

    m_xhat = xhat.clone();
    m_Phat = Phat.clone();
}

double c_ekf::AngDiff(double rad)
{
    double retVal;
    double n;
    double cfmod;
    n = floor((rad+M_PI)/(2.0*M_PI));
    cfmod = (rad+M_PI) - n*(2.0*M_PI);

    retVal = cfmod - M_PI;

    return retVal;
}
