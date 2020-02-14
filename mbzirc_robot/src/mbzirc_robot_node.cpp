// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// Include ROS message_type which will be published
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <math.h>

#define RAD_DEG     57.2957951
#define M_PI        3.1415926535897932384626433832795

class MBZIRC_ROBOT
{
    public:
        MBZIRC_ROBOT(ros::NodeHandle& n);        
        ~MBZIRC_ROBOT();
        void odom_publisher();
        void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void wheel_encoder_FL_callback(const std_msgs::Int32::ConstPtr& msg); //Wheel hub: Front Left                 
        void wheel_encoder_FR_callback(const std_msgs::Int32::ConstPtr& msg); //Wheel hub: Front Right
        void wheel_encoder_RL_callback(const std_msgs::Int32::ConstPtr& msg); //Wheel hub: Rear Left
        void wheel_encoder_RR_callback(const std_msgs::Int32::ConstPtr& msg); //Wheel hub: Rear Right
        void dynamixel_position_callback(const sensor_msgs::JointState::ConstPtr& msg); //Position(yaw angle) of each wheel
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    private:        
        ros::NodeHandle nh;

        //Odometry
        ros::Publisher odom_pub;
        nav_msgs::Odometry odom_msg;
        double dt;
        double x_stack = 0;
        double y_stack = 0;
        double yaw_rate;
        double yaw_estimated = 0;
        bool rotational_motion;
        bool low_speed;
        //IMU
        ros::Subscriber imu_sub;
        sensor_msgs::Imu imu_msg;
        //Odrive_Motor_Hub 
        ros::Subscriber wheel_speed_FL_sub;
        ros::Subscriber wheel_speed_FR_sub;
        ros::Subscriber wheel_speed_RL_sub;
        ros::Subscriber wheel_speed_RR_sub;
        double FL_linear_speed; // unit : [m/s]
        double FR_linear_speed;
        double RL_linear_speed;
        double RR_linear_speed;
        double wheel_radius = 0.115;
        // Dynamixel_Motor
        ros::Subscriber dynamixel_position_sub;
        double FL_yaw_angle; // unit : [rad]
        double FR_yaw_angle;
        double RL_yaw_angle;
        double RR_yaw_angle;
        double angle_direction =  -1.0; // Counter-clock direction : + --> if not, change it to "-1"
        
        //cmd velocity
        ros::Subscriber cmd_vel_sub;
        double cmd_x_vel;
        double cmd_y_vel;
        double cmd_yaw_rate;

        // Velocity test
        // double v, v_f, v_r;
        // double beta;
        // double yaw_rate_test;
        // double delta_f, delta_r;
};

MBZIRC_ROBOT::MBZIRC_ROBOT(ros::NodeHandle& n) 
{
    // Publisher/s
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 20);
    imu_sub = nh.subscribe("/gx5/imu/data", 10, &MBZIRC_ROBOT::imu_callback,this);
    //Odrive_Motor_Hub
    wheel_speed_FL_sub = nh.subscribe("/front/odrive/left/raw_odom/velocity", 10, &MBZIRC_ROBOT::wheel_encoder_FL_callback,this);
    wheel_speed_FR_sub = nh.subscribe("/front/odrive/right/raw_odom/velocity", 10, &MBZIRC_ROBOT::wheel_encoder_FR_callback,this);
    wheel_speed_RL_sub = nh.subscribe("/back/odrive/left/raw_odom/velocity", 10, &MBZIRC_ROBOT::wheel_encoder_RL_callback,this);
    wheel_speed_RR_sub = nh.subscribe("/back/odrive/right/raw_odom/velocity", 10, &MBZIRC_ROBOT::wheel_encoder_RR_callback,this);         
    // Dynamixel Motor
    dynamixel_position_sub = nh.subscribe("/dynamixel_workbench/joint_states", 10, &MBZIRC_ROBOT::dynamixel_position_callback,this);
    // Command velocity
    cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &MBZIRC_ROBOT::cmd_vel_callback, this);
    ROS_DEBUG("mbzric_robot publisher created. And, this node published odometry of mbzric robot");
};

MBZIRC_ROBOT::~MBZIRC_ROBOT() 
{    
    ROS_INFO("MBZIRC_ROBOT destructor.");
}

void MBZIRC_ROBOT::odom_publisher()
{
    ros::Rate r(20);
    dt = 0.05;
    double rotaional_cmd_vel_thres = 0.03;
    double x_delta;
    double y_delta;
    // Determine rotational or translational motion
    if (RR_yaw_angle * RL_yaw_angle < 0 && RL_linear_speed * RR_linear_speed < 0 ){ 
        rotational_motion = true; //Rotational motion in the same place
    }
    else{
        rotational_motion = false; //Translational motion
    }

    if (sqrt(cmd_x_vel*cmd_x_vel + cmd_y_vel*cmd_y_vel) < rotaional_cmd_vel_thres){ 
        low_speed = true; //Low speed
    }
    else{
        low_speed = false; 
    } 

    odom_msg.header.stamp = imu_msg.header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    tf2::Quaternion quat;
    geometry_msgs::Quaternion quat_msg;
    if (rotational_motion ==false ){
        x_delta = 0.5 * (RL_linear_speed + RR_linear_speed) * cos(0.5*(RL_yaw_angle + RR_yaw_angle)) * dt;
        y_delta = 0.5 * (RL_linear_speed + RR_linear_speed) * sin(0.5*(RL_yaw_angle + RR_yaw_angle)) * dt;
        odom_msg.twist.twist.linear.x = 0.5 * (RL_linear_speed + RR_linear_speed) * cos(0.5*(RL_yaw_angle + RR_yaw_angle));
        odom_msg.twist.twist.linear.y = 0.5 * (RL_linear_speed + RR_linear_speed) * sin(0.5*(RL_yaw_angle + RR_yaw_angle));
        // ROS_INFO("1");
        // // odom_msg.pose.pose.orientation = odom_msg.pose.pose.orientation + quat_msg;
    }
    else if(rotational_motion == true){
        x_delta = 0;
        y_delta = 0;
        odom_msg.twist.twist.linear.x = 0.5 * (RL_linear_speed + RR_linear_speed) * cos(0.5*(RL_yaw_angle + RR_yaw_angle));
        odom_msg.twist.twist.linear.y = 0.5 * (RL_linear_speed + RR_linear_speed) * sin(0.5*(RL_yaw_angle + RR_yaw_angle));
        // ROS_INFO("2");
    }
    else if(low_speed == true){
        x_delta = 0;
        y_delta = 0;
        odom_msg.twist.twist.linear.x = 0;
        odom_msg.twist.twist.linear.y = 0;
        // ROS_INFO("2");
    }

    odom_msg.twist.twist.angular.x = imu_msg.angular_velocity.x;
    odom_msg.twist.twist.angular.y = imu_msg.angular_velocity.y;
    odom_msg.twist.twist.angular.z = imu_msg.angular_velocity.z;

    yaw_estimated = yaw_estimated + odom_msg.twist.twist.angular.z * dt;
    quat.setRPY(0,0,yaw_estimated);
    quat.normalize();
    quat_msg = tf2::toMsg(quat);

    x_stack = x_stack +  x_delta * cos(yaw_estimated) - y_delta * sin(yaw_estimated);
    y_stack = y_stack +  x_delta * sin(yaw_estimated) + y_delta * cos(yaw_estimated);

    odom_msg.pose.pose.position.x = x_stack;
    odom_msg.pose.pose.position.y = y_stack;

    odom_msg.pose.pose.orientation.x = quat_msg.x;
    odom_msg.pose.pose.orientation.y = quat_msg.y;
    odom_msg.pose.pose.orientation.z = quat_msg.z;
    odom_msg.pose.pose.orientation.w = quat_msg.w;

    odom_pub.publish(odom_msg);
    r.sleep();

};

void MBZIRC_ROBOT::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    //READ FROM TRAM_MISSION
    imu_msg.header = msg -> header;
    imu_msg.linear_acceleration.x = msg -> linear_acceleration.x;
    imu_msg.linear_acceleration.y = msg -> linear_acceleration.y;
    imu_msg.linear_acceleration.z = msg -> linear_acceleration.z; 
    imu_msg.angular_velocity.x = msg -> angular_velocity.x;
    imu_msg.angular_velocity.y = msg -> angular_velocity.y;
    imu_msg.angular_velocity.z = msg -> angular_velocity.z;

    MBZIRC_ROBOT::odom_publisher();
}

//Wheel hub: Front Left  
void MBZIRC_ROBOT::wheel_encoder_FL_callback(const std_msgs::Int32::ConstPtr& msg) {
    int FL_encoder = msg -> data;
    FL_linear_speed = (double)FL_encoder * 2 * M_PI * wheel_radius / 90.0 ;
    // ROS_INFO("Linear Speed: (%f, %f, %f, %f)", FL_linear_speed,FR_linear_speed,RL_linear_speed,RR_linear_speed);
}
//Wheel hub: Front Right  
void MBZIRC_ROBOT::wheel_encoder_FR_callback(const std_msgs::Int32::ConstPtr& msg) {
    int FR_encoder = msg -> data;
    FR_linear_speed = (-1) * (double)FR_encoder * 2 * M_PI *wheel_radius / 90.0; // (-1) : Right data is reversed 
}
//Wheel hub: Rear Left  
void MBZIRC_ROBOT::wheel_encoder_RL_callback(const std_msgs::Int32::ConstPtr& msg) {
    int RL_encoder = msg -> data;
    RL_linear_speed = (double)RL_encoder * 2 * M_PI *wheel_radius / 90.0;
}
//Wheel hub: Rear Right  
void MBZIRC_ROBOT::wheel_encoder_RR_callback(const std_msgs::Int32::ConstPtr& msg) {
    int RR_encoder = msg -> data;
    RR_linear_speed = (-1) * (double)RR_encoder * 2 * M_PI *wheel_radius / 90.0; //(-1) : Right data is reversed 

    
}
//Dynamixel Yaw angle
void MBZIRC_ROBOT::dynamixel_position_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    //Matching position data to robot system --> it is notified by Donghee.
    RR_yaw_angle = msg -> position.at(0) * angle_direction; // unit : [rad] //1
    FL_yaw_angle = msg -> position.at(1) * angle_direction; //2
    FR_yaw_angle = msg -> position.at(2) * angle_direction; //3
    RL_yaw_angle = msg -> position.at(3) * angle_direction;
    double FL_yaw_deg = FL_yaw_angle * RAD_DEG;
    double FR_yaw_deg = FR_yaw_angle * RAD_DEG;
    double RL_yaw_deg = RL_yaw_angle * RAD_DEG;
    double RR_yaw_deg = RR_yaw_angle * RAD_DEG;

    // double test_1 = RAD_DEG* msg -> position.at(0) * angle_direction;
    // double test_2 = RAD_DEG* msg -> position.at(1) * angle_direction;
    // double test_3 = RAD_DEG* msg -> position.at(2) * angle_direction;
    // ROS_INFO("Yaw angle: (%f, %f, %f, %f, %f, %f, %f)", test_1, test_2, test_3, FL_yaw_deg,FR_yaw_deg,RL_yaw_deg,RR_yaw_deg);
}
//cmd velocity callback
void MBZIRC_ROBOT::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_x_vel = msg -> linear.x;
    cmd_y_vel = msg -> linear.y;
    cmd_yaw_rate = msg -> angular.z; 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbzric_robot");
    ros::NodeHandle nh;   

    // ros::Rate r(20);
    
    MBZIRC_ROBOT _mbzric_odom(nh);

    while(ros::ok()){
        // _mbzric_odom.odom_publisher();
        ros::spin();
        // ros::spinOnce();
    }
    return 0;
}
