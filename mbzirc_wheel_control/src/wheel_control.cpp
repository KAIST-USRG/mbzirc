#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

double const RESOLUTION = 1002866;
const double RAD_DEG = 57.2957951;
int dynamixel_degree = 0;
int dynamixel_degree_lf = 0;
int dynamixel_degree_rf = 0;
int dynamixel_degree_lb = 0;
int dynamixel_degree_rb = 0;
double point_turn_angle = atan2(1055, 655) * RAD_DEG;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
  if(twist_msg->angular.z > 0.3 || twist_msg->angular.z < -0.3){
    dynamixel_degree = point_turn_angle / 360.0 * RESOLUTION; 
    if(dynamixel_degree > 501433) dynamixel_degree = -501433 + dynamixel_degree - 501433;

    dynamixel_degree_lf = -dynamixel_degree;
    dynamixel_degree_rf = -dynamixel_degree;
    dynamixel_degree_lb = dynamixel_degree;
    dynamixel_degree_rb = dynamixel_degree;
  }
  else{
    double x = twist_msg->linear.x;
    double y = -twist_msg->linear.y;
    double atan_degree = atan2(y, x) * RAD_DEG;
    dynamixel_degree = atan_degree / 360.0 * RESOLUTION; 
    double speed = std::sqrt(x*x + y*y);
    if(speed < 0.001 && speed > -0.001) dynamixel_degree = 0;
    else if(dynamixel_degree > 501433) dynamixel_degree = -501433 + dynamixel_degree - 501433;
    dynamixel_degree_lf
    = dynamixel_degree_rf
    = dynamixel_degree_lb
    = dynamixel_degree_rb
    = dynamixel_degree;
  }
  ROS_DEBUG("dynamixel_input: %d", dynamixel_degree);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");

  ros::NodeHandle n;
  ros::service::waitForService("/dynamixel_workbench/dynamixel_command", -1);
  ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);

  dynamixel_workbench_msgs::DynamixelCommand srv;

  std::string item_command = "";
  std::string item_addr = "Goal_Position";

  while(ros::ok()) {

    srv.request.command = item_command;
    srv.request.id = 1;
    srv.request.addr_name = item_addr;
    srv.request.value = dynamixel_degree_rb;

    if (client.call(srv))
    {
      ROS_DEBUG("send ID and Position Value : %u, %d", (uint8_t)srv.request.id, (int32_t)srv.request.value);
      ROS_DEBUG("receive result : %d", (bool)srv.response.comm_result);
    }
    else
    {
      ROS_ERROR("Failed to call dynamixel_command");
      return 1;
    }

    srv.request.command = item_command;
    srv.request.id = 2;
    srv.request.addr_name = item_addr;
    srv.request.value = dynamixel_degree_lf;

    if (client.call(srv))
    {
      ROS_DEBUG("send ID and Position Value : %u, %d", (uint8_t)srv.request.id, (int32_t)srv.request.value);
      ROS_DEBUG("receive result : %d", (bool)srv.response.comm_result);
    }
    else
    {
      ROS_ERROR("Failed to call dynamixel_command");
      return 1;
    }

    srv.request.command = item_command;
    srv.request.id = 3;
    srv.request.addr_name = item_addr;
    srv.request.value = dynamixel_degree_rf;

    if (client.call(srv))
    {
      ROS_DEBUG("send ID and Position Value : %u, %d", (uint8_t)srv.request.id, (int32_t)srv.request.value);
      ROS_DEBUG("receive result : %d", (bool)srv.response.comm_result);
    }
    else
    {
      ROS_ERROR("Failed to call dynamixel_command");
      return 1;
    }

    srv.request.command = item_command;
    srv.request.id = 4;
    srv.request.addr_name = item_addr;
    srv.request.value = dynamixel_degree_lb;

    if (client.call(srv))
    {
      ROS_DEBUG("send ID and Position Value : %u, %d", (uint8_t)srv.request.id, (int32_t)srv.request.value);
      ROS_DEBUG("receive result : %d", (bool)srv.response.comm_result);
    }
    else
    {
      ROS_ERROR("Failed to call dynamixel_command");
      return 1;
    }

    ros::spinOnce();
  }

  return 0;
}
