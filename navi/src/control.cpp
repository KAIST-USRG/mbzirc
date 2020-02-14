#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>
#include <iterator>
#include "tf/transform_datatypes.h"
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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

// #include <vision_msgs/Detection2DArray.h>

class Control
{
private:
    ros::Publisher pub_cmd;
    ros::Publisher vector_direction_cmd;
    ros::Publisher heading_direction_cmd;
    ros::Subscriber ekf_odom;
    ros::Subscriber goal;
    ros::Subscriber outdoor_srv_running_sub;
    ros::NodeHandle nh;
    float yaw_x;
    float yaw_y;
    float yaw_x_;
    float yaw_y_;
    bool move_to_goal = false;
    float x_vel_vec;
    float x_vel_vec_prev;
    float y_vel_vec_prev;
    float angular_z_prev;
    float y_vel_vec;
    float x_goal_vec;
    float y_goal_vec;
    float x_ref =1;
    float y_ref =0;
    float aco;
    float co;
    float move_heading;
    float current_point_x;
    float current_point_y;
    float current_point_z;
    float goal_point_x = 0;
    float goal_point_y = 0;
    float goal_point_z = 0;
    float current_point_orientation_x;
    float current_point_orientation_y;
    float current_point_orientation_z;
    float current_point_orientation_w;
    float goal_point_orientation_x =0;
    float goal_point_orientation_y =0;
    float goal_point_orientation_z =0;
    float goal_point_orientation_w =1;
    float current_heading;
    float goal_heading;
    float max_x_vel =1.5;
    float scailing_factor;
    float det;
    float dot;
    double roll, pitch, yaw, roll_, pitch_, yaw_;
    double heading_local;
    double weight = 0.05;
    int len=0;
    int i=0;
    int k=0;
    bool outdoor_srv_running;

public:
    Control()
    { 
        pub_cmd  = nh.advertise<geometry_msgs::Twist>("/move_base/cmd_vel",100);
        vector_direction_cmd  = nh.advertise<geometry_msgs::PoseStamped>("/check_vector",100);
        heading_direction_cmd  = nh.advertise<geometry_msgs::PoseStamped>("/check_cmd_heading",100);
        ekf_odom = nh.subscribe("/outdoor_nav/odometry/EKF_estimated",1,&Control::ekf_odom_callback, this);
        goal = nh.subscribe("/goal_point", 1, &Control::goal_point_callback, this);
        outdoor_srv_running_sub = nh.subscribe("/outdoor_srv_running", 1, &Control::outdoor_srv_running_callback, this);
        

    }
    void clockwise_rotation(float x, float y, float angle, float move_heading){
        // x_vel_vec = cos(angle)*x + sin(angle)*y;
        // y_vel_vec = -sin(angle)*x + cos(angle)*y;
 
        x_vel_vec = cos(move_heading-angle);
        y_vel_vec = sin(move_heading-angle);
        geometry_msgs::PoseStamped posestamp;
        geometry_msgs::PoseStamped heading_dir;
        // goal_point_x
        posestamp.header.stamp = ros::Time::now();
        posestamp.header.frame_id = "/base_footprint";
        double x_elem = (goal_point_x - current_point_x)*cos(yaw) + (goal_point_y - current_point_y)*sin(yaw);
        double y_elem = -(goal_point_x - current_point_x)*sin(yaw) + (goal_point_y - current_point_y)*cos(yaw);
        // double x_elem = (goal_point_x - current_point_x)*cos(move_heading-angle) + (goal_point_y - current_point_y)*sin(move_heading-angle);
        // double y_elem = -(goal_point_x - current_point_x)*sin(move_heading-angle) + (goal_point_y - current_point_y)*cos(move_heading-angle);
        posestamp.pose.position.x = 0;
        posestamp.pose.position.y = 0;
        // double heading_cur = atan2(y_vel_vec, x_vel_vec);
        heading_local = atan2(y_elem, x_elem);
        tf2::Quaternion quat_ekf;
        geometry_msgs::Quaternion quat_ekf_msg;

        if (std::isnan(heading_local)){
            heading_local = 0;
        }
        // quat_ekf.setRPY(0,0,heading_local + yaw);
        quat_ekf.setRPY(0,0,heading_local);
        quat_ekf.normalize();
        quat_ekf_msg = tf2::toMsg(quat_ekf);
        posestamp.pose.orientation= quat_ekf_msg;
        vector_direction_cmd.publish(posestamp);


        heading_dir.header.stamp = ros::Time::now();
        heading_dir.header.frame_id = "/base_footprint";
        heading_dir.pose.position.x = 0;
        heading_dir.pose.position.y = 0;
        double heading_dir_cur = atan2(y_vel_vec, x_vel_vec);
        tf2::Quaternion quat_head;
        geometry_msgs::Quaternion quat_head_msg;

        if (std::isnan(heading_dir_cur)){
            heading_dir_cur = 0;
        }
        // quat_ekf.setRPY(0,0,heading_cur + yaw);
        quat_head.setRPY(0,0,heading_dir_cur);
        quat_head.normalize();
        quat_head_msg = tf2::toMsg(quat_head);
        heading_dir.pose.orientation= quat_head_msg;
        heading_direction_cmd.publish(heading_dir);

    }
    void ekf_odom_callback(const nav_msgs::Odometry& msg){
        current_point_x = msg.pose.pose.position.x;
        current_point_y = msg.pose.pose.position.y;
        current_point_z = msg.pose.pose.position.z;
        current_point_orientation_x = msg.pose.pose.orientation.x;
        current_point_orientation_y = msg.pose.pose.orientation.y;
        current_point_orientation_z = msg.pose.pose.orientation.z;
        current_point_orientation_w = msg.pose.pose.orientation.w;
        tf::Quaternion q(current_point_orientation_x,current_point_orientation_y,current_point_orientation_z,current_point_orientation_w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        current_heading = yaw;
        x_goal_vec = goal_point_x - current_point_x;
        y_goal_vec = goal_point_y - current_point_y;
        move_heading = atan2(y_goal_vec, x_goal_vec);
        clockwise_rotation(x_goal_vec, y_goal_vec, yaw, move_heading);
        if (fabs(x_vel_vec) >= fabs(y_vel_vec)){
            scailing_factor = max_x_vel / fabs(x_vel_vec);
        }
        else{
            scailing_factor = max_x_vel / fabs(y_vel_vec);
        }
        x_vel_vec = scailing_factor * x_vel_vec;
        y_vel_vec = scailing_factor * y_vel_vec;
        yaw_x = cos(yaw);
        yaw_y = sin(yaw);
        dot = yaw_x * yaw_x_ + yaw_y * yaw_y_;
        det = yaw_x_ * yaw_y - yaw_y_*yaw_x;
        co = dot;
        aco = acos(co)*180/M_PI;
        if (det>0){
            aco =360-aco;
        }
        geometry_msgs::Twist vel;
        vel.linear.x = x_vel_vec;
        if (fabs(y_vel_vec) > 0.1){
            vel.linear.y = y_vel_vec;
        }
        else{
            vel.linear.y = 0;
        }


        //yaw rate  pid-controller

        double error =   goal_heading - current_heading; //rad
        if (error > 3.14){
            error = error -2 * 3.14;
        }
        else if (error < -3.14){
            error = error  + 2 * 3.14;
        }
        double error_stack = error_stack + error * 0.05;
        double max_yaw_rate = 0.4;
        if (fabs(error) < 1 / 57.3 || sqrt(pow(msg.twist.twist.linear.x,2)+ pow(msg.twist.twist.linear.y,2)) < 0.01){
            error_stack = 0;
        }

        if (error_stack > 3.14){
            error_stack = 3.14;
        }
        else if (error_stack < -3.14){
            error_stack = -3.14;
        }
        
        double P_yaw_rate = 1.2;
        double I_yaw_rate = 0.2;
        double cmd_out = P_yaw_rate * error + I_yaw_rate * error_stack;
   
        cmd_out = P_yaw_rate * error + I_yaw_rate * error_stack; //
        if (cmd_out > max_yaw_rate){
            cmd_out = max_yaw_rate;
        }
        else if(cmd_out < -max_yaw_rate){
            cmd_out = -max_yaw_rate;
        }
        // ROS_INFO("Heading error: %f, error stack: %f, CMD: %f", error,error_stack, cmd_out);


        vel.angular.z = cmd_out;
        if (fabs(vel.angular.z) > 0.2){
            vel.linear.x = 0.5 * vel.linear.x;
            vel.linear.y = 0.5 * vel.linear.y;
        }

        // if (aco >=355 && aco <=5 ){
        //     vel.angular.z = 0.0;
        // }
        // else if ((aco >5 && acos<30)) {
        //     vel.angular.z = +0.3;
        // }
        // else if ((aco >=30 && acos<180)) {
        //     vel.angular.z = +0.6;
        // }
        // else if (aco >=180 && aco <330){
        //     vel.angular.z = -0.6;
        // }
        // else if (aco >=330 && aco <355){
        //     vel.angular.z = -0.3;
        // }
        // else {
        //     vel.angular.z =0.0;
        // }
        // if (fabs(vel.angular.z) >= 0.4){
        //     vel.linear.x = 0.5 * vel.linear.x;
        //     vel.linear.y = 0.5 * vel.linear.y;
        // }

        // // vel.linear.x = vel.linear.x *(1-weight) + (weight) * x_vel_vec_prev;
        // // vel.linear.y = vel.linear.y *(1-weight) + (weight) * y_vel_vec_prev;
        // vel.angular.z = vel.angular.z * (1-weight) + (weight) * angular_z_prev;


        // if (move_to_goal == true){
        //     pub_cmd.publish(vel);
        // }
        // // if ((sqrt(pow((goal_point_x-current_point_x),2)+pow((goal_point_y-current_point_y),2))) < 2 && (sqrt(pow((goal_point_x-current_point_x),2)+pow((goal_point_y-current_point_y),2))) > 1){
        // //     move_to_goal = false;
        // // }
        // if ((sqrt(pow((goal_point_x-current_point_x),2)+pow((goal_point_y-current_point_y),2))) < 1){
        //     move_to_goal = false;        
        //     vel.linear.x = 0;
        //     vel.linear.y = 0;
        //     vel.angular.z = 0;
        //     pub_cmd.publish(vel);
        // }

        bool move_due_to_outdoor_srv_running = false;
        if (outdoor_srv_running == true){
            move_due_to_outdoor_srv_running = outdoor_srv_running;
        }
        
        if (move_due_to_outdoor_srv_running == true){
            pub_cmd.publish(vel);

            if ((sqrt(pow((goal_point_x-current_point_x),2)+pow((goal_point_y-current_point_y),2))) < 1){
                move_to_goal = false;        
                vel.linear.x = 0;
                vel.linear.y = 0;
                vel.angular.z = 0;
                pub_cmd.publish(vel);
                }
        }

        x_vel_vec_prev = vel.linear.x;
        y_vel_vec_prev = vel.linear.y;
        angular_z_prev = vel.angular.z;
        // std::cout<<move_to_goal<<std::endl;


    }
    void goal_point_callback(const geometry_msgs::PoseStamped& msg){
        goal_point_x = msg.pose.position.x;
        goal_point_y = msg.pose.position.y;
        goal_point_z = msg.pose.position.z;
        goal_point_orientation_x = msg.pose.orientation.x;
        goal_point_orientation_y = msg.pose.orientation.y;
        goal_point_orientation_z = msg.pose.orientation.z;
        goal_point_orientation_w = msg.pose.orientation.w;
        tf::Quaternion q_(goal_point_orientation_x,goal_point_orientation_y,goal_point_orientation_z,goal_point_orientation_w);
        tf::Matrix3x3 m_(q_);
        m_.getRPY(roll_, pitch_, yaw_);
        goal_heading = yaw_;
        yaw_x_ = cos(yaw_);
        yaw_y_ = sin(yaw_);
        move_to_goal = true;
        ROS_INFO("&d", move_to_goal);
    }

    void outdoor_srv_running_callback(const std_msgs::Bool& msg){
        outdoor_srv_running = msg.data;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "Control");
    Control cont;
    ros::spin();
    

    return 0;
}