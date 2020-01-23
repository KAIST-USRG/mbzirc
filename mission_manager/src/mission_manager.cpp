#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include "mission_manager/test.h"
#include <mission_manager/ugv_move.h>
#include <mission_manager/ur_move.h>
#include "mission.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

// srv
#include "mission_manager/ur_move.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "mission_manager_node");
  ros::NodeHandle n;

  // Load mission preset param
  Mission_preset mission_preset;

  // ros::ServiceClient client_ugv = n.serviceClient<mission_manager::ugv_move>("ugv_move");
  // ros::service::waitForService("/dynamixel_workbench/dynamixel_command", -1);
  // ros::ServiceClient client_conveyor = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::ServiceClient client_ur = n.serviceClient<mission_manager::ur_move>("/mbzirc_ur5_control/ur_move");

  // mission_manager::ugv_move move_srv;
  // move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
  // move_srv.request.goal = mission_preset.get_brick_pos_tmp();
  // move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
  // move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();


  // dynamixel_workbench_msgs::DynamixelCommand conveyor_srv;
  // // LEFT CONVEYOR SRV
  // conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
  // conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
  // conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
  // conveyor_srv.request.value = mission_preset.get_conveyor_left_load_vel();

  // // RIGHT CONVEYOR SRV
  // conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
  // conveyor_srv.request.value = mission_preset.get_conveyor_right_load_vel();

  mission_manager::ur_move ur_srv;
  mission_preset.set_ur_action(UR_ACTION_LOAD);


  ur_srv.request.target_node_name = "service test";
  ur_srv.request.target_load_or_unload = UR_ACTION_LOAD;
  ur_srv.request.target_brick_color_code = ORANGE;
  ur_srv.request.target_brick_container_side_left_right = LEFT;
  
  
  ros::service::waitForService("/mbzirc_ur5_control/ur_move");
  
  if(client_ur.call(ur_srv)) {
    if (ur_srv.response.success_or_fail == false){
      std::cout << "Response : " << ur_srv.response.success_or_fail << std::endl;
    }
  
  }
  else
  {
    std::cout << "Failed to call service, request again..." << std::endl;
  }


  // if(client_ugv.call(move_srv)) {
  // std::cout << "Response : " << move_srv.response.success_or_fail << std::endl;
  // }
  // else
  // {
  //   std::cout << "Failed to call service, request again..." << std::endl;
  // }
  

  ///////////////////////////////////////////////// REGISTER MISSION SEQUENCE ///////////////////////////////
  // full seqence : Ready -> Move to bricks -> Load all orange brick -> Go to segment -> Unload orange bricks
  //                -> Go to bricks -> Load multi-color bricks -> Go to segment -> Unload multi-color bricks -> Finish

  // vector<mission_struct> mission_seq;
  // for(int mission_order = 0; mission_order < 9; mission_order++) {
  //   mission_struct tmp;

  //   if(mission_order == 0) {
  //     tmp.mission_name = "Ready";
  //     tmp.mission_type = 0;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 1) {
  //     tmp.mission_name = "Move_to_bricks";
  //     tmp.mission_type = 1;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 2) {
  //     tmp.mission_name = "Load_orange_bricks";
  //     tmp.mission_type = 2;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 3) {
  //     tmp.mission_name = "Move_to_segments";
  //     tmp.mission_type = 1;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 4) {
  //     tmp.mission_name = "Unload_orange_bricks";
  //     tmp.mission_type = 3;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 5) {
  //     tmp.mission_name = "Move_to_bricks";
  //     tmp.mission_type = 1;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 6) {
  //     tmp.mission_name = "Load_multicolor_bricks";
  //     tmp.mission_type = 2;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 7) {
  //     tmp.mission_name = "Move_to_segments";
  //     tmp.mission_type = 1;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 8) {
  //     tmp.mission_name = "Unload_muticolor_bricks";
  //     tmp.mission_type = 3;

  //     mission_seq.push_back(tmp);
  //   }
  //   else if(mission_order == 9) {
  //     tmp.mission_name = "Finish";
  //     tmp.mission_type = 5;

  //     mission_seq.push_back(tmp);
  //   }
  // }

  // // GOAL Initialization
  // move_base_msgs::MoveBaseGoal goal_mission2_brick,goal_mission2_segment;

  // goal_mission2_brick.target_pose.header.frame_id = "odom";
  // goal_mission2_brick.target_pose.header.stamp = ros::Time::now();
  // goal_mission2_brick.target_pose.pose.position.x = 1.0;
  // goal_mission2_brick.target_pose.pose.position.y = 10.0;
  // goal_mission2_brick.target_pose.pose.position.z = 0.0;
  // goal_mission2_brick.target_pose.pose.orientation.x = 0.0;
  // goal_mission2_brick.target_pose.pose.orientation.y = 0.0;
  // goal_mission2_brick.target_pose.pose.orientation.z = 0.0;
  // goal_mission2_brick.target_pose.pose.orientation.w = 0.0;

  // goal_mission2_segment.target_pose.header.frame_id = "odom";
  // goal_mission2_segment.target_pose.header.stamp = ros::Time::now();
  // goal_mission2_segment.target_pose.pose.position.x = 15.0;
  // goal_mission2_segment.target_pose.pose.position.y = 1.0;
  // goal_mission2_segment.target_pose.pose.position.z = 0.0;
  // goal_mission2_segment.target_pose.pose.orientation.x = 0.0;
  // goal_mission2_segment.target_pose.pose.orientation.y = 0.0;
  // goal_mission2_segment.target_pose.pose.orientation.z = 0.0;
  // goal_mission2_segment.target_pose.pose.orientation.w = 0.0;






  ros::spin();

  return 0;
}
