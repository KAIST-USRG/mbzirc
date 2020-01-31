#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
// #include "mission_manager/test.h"
#include "mbzirc_msgs/ugv_move.h"
#include "mbzirc_msgs/ur_move.h"
#include "mission.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "mission_manager_node");
  ros::NodeHandle n;

  // Load mission preset param
  Mission_preset mission_preset;

  ros::ServiceClient client_ugv_gps = n.serviceClient<mbzirc_msgs::ugv_move>("ugv_move_gps");
  ros::ServiceClient client_ugv_local = n.serviceClient<mbzirc_msgs::ugv_move>("ugv_move_local")
  
  ;
  ros::ServiceClient client_conveyor = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  // ros::ServiceClient client_ur = n.serviceClient<mission_manager::ur_move>("ur_move");
  ros::ServiceClient client_ur = n.serviceClient<mbzirc_msgs::ur_move>("/mbzirc_ur5_control/ur_move");
  // ros::service::waitForService("/dynamixel_workbench/dynamixel_command", -1);

  mbzirc_msgs::ugv_move move_srv;
  move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
  move_srv.request.goal = mission_preset.get_brick_pos_tmp();
  move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
  move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();


  dynamixel_workbench_msgs::DynamixelCommand conveyor_srv;
  // LEFT CONVEYOR SRV
  conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
  conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
  conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
  conveyor_srv.request.value = mission_preset.get_conveyor_left_load_vel();
  // RIGHT CONVEYOR SRV
  conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
  conveyor_srv.request.value = mission_preset.get_conveyor_right_load_vel();

  mbzirc_msgs::ur_move ur_srv;
  

  ///////////////////////////////////////////////// REGISTER MISSION SEQUENCE ///////////////////////////////
  // full seqence : 
  // Ready                      -> 0
  // Move to bricks             -> 1
  // Load all orange brick      -> 2
  // Go to segment              -> 3
  // Unload orange bricks       -> 4
  // Go to bricks               -> 5
  // Load multi-color bricks    -> 6
  // Go to segment              -> 7 
  // Unload multi-color bricks  -> 8
  // Finish                     -> 9

  vector<mission_struct> mission_seq;

  int test_start_state_idx = 2;
  int test_end_state_idx = 3;

  for(int mission_order = test_start_state_idx; mission_order < test_end_state_idx; mission_order++) {
    mission_struct tmp;

    if(mission_order == 0) {
      // tmp.mission = "Ready";
      tmp.mission = MISSION_READY;
      tmp.mission_type = 0;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 1) {
      // tmp.mission = "Move_to_bricks";
      tmp.mission = MISSION_MOVE_TO_BRICK;
      tmp.mission_type = 1;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 2) {
      // tmp.mission = "Load_orange_bricks";
      tmp.mission = MISSION_LOAD_ORANGE_BRICKS;
      tmp.mission_type = 2;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 3) {
      // tmp.mission = "Move_to_segments";
      tmp.mission = MISSION_MOVE_TO_SEGMENT;
      tmp.mission_type = 1;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 4) {
      // tmp.mission = "Unload_orange_bricks";
      tmp.mission = MISSION_UNLOAD_ORANGE_BRICKS;
      tmp.mission_type = 3;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 5) {
      // tmp.mission = "Move_to_bricks";
      tmp.mission = MISSION_MOVE_TO_BRICK;
      tmp.mission_type = 1;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 6) {
      // tmp.mission = "Load_multicolor_bricks";
      tmp.mission = MISSION_LOAD_MULTICOLOR_BRICKS;
      tmp.mission_type = 2;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 7) {
      // tmp.mission = "Move_to_segments";
      tmp.mission = MISSION_MOVE_TO_SEGMENT;
      tmp.mission_type = 1;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 8) {
      // tmp.mission = "Unload_muticolor_bricks";
      tmp.mission = MISSION_UNLOAD_MULTICOLOR_BRICKS;
      tmp.mission_type = 3;

      mission_seq.push_back(tmp);
    }
    else if(mission_order == 9) {
      // tmp.mission = "Finish";
      tmp.mission = MISSION_FINISH;
      tmp.mission_type = 5;

      mission_seq.push_back(tmp);
    }
  }

  ///////////////////////////////////////////////// STATE MACHINE RUNNING ///////////////////////////////
  int orange_load_left_cnt = 0;
  int orange_load_right_cnt = 0;
  int blue_load_cnt = 0;
  int green_load_cnt = 0;
  int red_load_cnt = 0;

  bool orange_load_all_done_flg = false;
  bool orange_load_left_done_flg = false;
  bool orange_load_right_done_flg = false;
  bool orange_unload_done_flg = false;
  bool orange_unload_left_done_flg = false;
  bool orange_unload_right_done_flg = false;
  bool orange_left_align_flg = false;
  bool orange_right_align_flg = false;

  bool blue_load_done_flg = false;
  bool blue_unload_done_flg = false;

  bool green_load_done_flg = false;
  bool green_unload_done_flg = false;

  bool red_load_done_flg = false;
  bool red_unload_done_flg = false;

  bool multicolor_load_done_flg = false;
  bool multicolor_unload_done_flg = false;
  
  bool state_finish_flg = false;

  for(int cur_mission_idx = 0; cur_mission_idx < mission_seq.size();) {
        
    int cur_mission = mission_seq[cur_mission_idx].mission;

    switch(cur_mission) {

      case MISSION_READY:
      {
        std::cout <<"=============MISSION_READY START=============" << std::endl;
        cur_mission_idx++;

        std::cout <<"=============MISSION_READY DONE=============" << std::endl;

        break;
      }

      case MISSION_MOVE_TO_BRICK:
      {
        std::cout <<"=========MISSION_MOVE_TO_BRICK START=========" << std::endl;

        move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
        move_srv.request.goal = mission_preset.get_brick_pos_tmp();
        move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
        move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
        move_srv.request.destination_name = BRICKS;

        if(client_ugv_gps.call(move_srv)) {
          if(move_srv.response.success_or_fail == true) {
            std::cout << "UGV ARRIVED AT BRICK SECTION" << std::endl;
          }
          else {
            std::cout << "UGV FAIL TO ARRIVE AT BRICK SECTION, TRY AGAIN..." << std::endl;
            break;
          }
        }
        else {
          std::cout << "Failed to call UGV GPS NODE (in charge of DEAGYU) service" << std::endl;
          return -1;
        }

        if(move_srv.response.success_or_fail == true)
          cur_mission_idx++;

        std::cout <<"=========MISSION_MOVE_TO_BRICK DONE=========" << std::endl;
        break;
      }

      case MISSION_LOAD_ORANGE_BRICKS:
      {
        // std::cout <<"=======MISSION_FIND_ORANGE_BRICKS START======" << std::endl;
        // {
        //   move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
        //   move_srv.request.goal = mission_preset.get_brick_pos_tmp();
        //   move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
        //   move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
        //   move_srv.request.destination_name = BRICKS;
        //   move_srv.request.target_color_brick = COLOR_ORANGE;
        //   move_srv.request.target_action = UGV_ACTION_LOAD;

        //   if(client_ugv_local.call(move_srv))
        //   {
        //     if(move_srv.response.success_or_fail == true) {
        //       std::cout << "UGV ARRIVED IN FRONT OF THE ORANGE BRICKS" << std::endl;
        //     }
        //     else {
        //       std::cout << "UGV FAIL TO ARRIVE IN FRONT OF THE ORANGE BRICKS, TRY AGAIN..." << std::endl;
        //       break;
        //     }
        //   }
        //   else
        //   {
        //     std::cout << "Failed to call UGV LOCAL NODE (in charge of DONGHEE) service" << std::endl;
        //     return -1;
        //   }
        // }
        std::cout <<"=======MISSION_FIND_ORANGE_BRICKS DONE======" << std::endl;



        std::cout <<"=======MISSION_LOAD_ORANGE_BRICKS START======" << std::endl;
        {
          //======================= for left container
          if(orange_load_left_done_flg == false) {
            
            mission_preset.set_ur_action(UR_ACTION_LOAD);
            mission_preset.set_ur_target_brick_color(COLOR_ORANGE);
            mission_preset.set_target_brick_container_side(CONTAINER_LEFT);

            for(int left_cnt = 0; left_cnt < 5;)
            {
              ur_srv.request.target_node_name = mission_preset.get_ur_target_node_name();
              ur_srv.request.target_load_or_unload = mission_preset.get_ur_action();
              ur_srv.request.target_brick_color_code = mission_preset.get_ur_target_brick_color();
              ur_srv.request.target_brick_container_side_left_right = mission_preset.get_target_brick_container_side();

              if(client_ur.call(ur_srv)) {
                std::cout << "Response : " << ur_srv.response.success_or_fail << std::endl;
                if(ur_srv.response.success_or_fail == true){
                  // load success
                  left_cnt++;
                  orange_load_left_cnt++;
                  if(orange_load_left_cnt>=5){
                    break;
                  }
                }
                else {
                  // unreachable or other reasons
                  break;
                }
              }
              else {
                std::cout << "Failed to call service" << std::endl;
                return -1;
              }
            }

            if(orange_load_left_cnt < 5) { // NOT FULLY LOADED
              std::cout << "ORAGNE BRICKS ARE NOT FULLY LOADED(LEFT), TRY AGAIN FROM FINDING BRICKS" << std::endl;
              orange_load_left_done_flg = false;
              break;
            }
            else{
              orange_load_left_done_flg = true;
            }
          }
          

          //======================= left conveyor move back for brick aligning
          if(orange_left_align_flg == false)
          {
            conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
            conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
            conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
            conveyor_srv.request.value = mission_preset.get_conveyor_left_load_vel();

            if(client_conveyor.call(conveyor_srv)) {
              std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
              if(conveyor_srv.response.comm_result == true) {
                sleep(mission_preset.get_conveyor_defualt_move_back_time());
                
                // conveyor stop
                conveyor_srv.request.value = 0;
                if(client_conveyor.call(conveyor_srv)) {
                  std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
                  orange_left_align_flg = true;
                }
                else {
                  std::cout << "Failed to call service" << std::endl;
                  return -1;
                }
              }
            }
            else
            {
              std::cout << "Failed to call service" << std::endl;
              return -1;
            }
          }
          
          //======================= for right container
          if(orange_load_right_done_flg == false) {
            mission_preset.set_ur_action(UR_ACTION_LOAD);
            mission_preset.set_ur_target_brick_color(COLOR_ORANGE);
            mission_preset.set_target_brick_container_side(CONTAINER_RIGHT);

            for(int right_cnt = 0; right_cnt < 5;){
              ur_srv.request.target_node_name = mission_preset.get_ur_target_node_name();
              ur_srv.request.target_load_or_unload = mission_preset.get_ur_action();
              ur_srv.request.target_brick_color_code = mission_preset.get_ur_target_brick_color();
              ur_srv.request.target_brick_container_side_left_right = mission_preset.get_target_brick_container_side();

              if(client_ur.call(ur_srv)) {
                std::cout << "Response : " << ur_srv.response.success_or_fail << std::endl;
                if(ur_srv.response.success_or_fail == true){
                  // load success
                  right_cnt++;
                  orange_load_right_cnt++;
                  if(orange_load_right_cnt>=5){
                    break;
                  }
                }
                else {
                  // unreachable or other reasons
                  break;
                }
              }
              else {
                std::cout << "Failed to call service" << std::endl;
                return -1;
              }
            }

            if(orange_load_right_cnt < 5) { // NOT FULLY LOADED
              std::cout << "ORAGNE BRICKS ARE NOT FULLY LOADED(RIGHT), TRY AGAIN FROM FINDING BRICKS" << std::endl;
              orange_load_right_done_flg = false;
              break;
            }
            else{
              orange_load_right_done_flg = true;
            }
          }

          //======================= right conveyor move back for brick aligning
          if(orange_right_align_flg == false)
          {
            conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
            conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
            conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
            conveyor_srv.request.value = mission_preset.get_conveyor_right_load_vel();

            if(client_conveyor.call(conveyor_srv)) {
              std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
              if(conveyor_srv.response.comm_result == true) {
                sleep(mission_preset.get_conveyor_defualt_move_back_time());
                
                //======================= conveyor stop
                conveyor_srv.request.value = 0;
                if(client_conveyor.call(conveyor_srv)) {
                  std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
                  orange_right_align_flg = true;
                }
                else {
                  std::cout << "Failed to call service" << std::endl;
                  return -1;
                }
              }
            }
            else
            {
              std::cout << "Failed to call service" << std::endl;
              return -1;
            }
          }
          
          //======================= ALL ORANGE BRICKS ARE LOADED
          if(orange_load_left_cnt + orange_load_right_cnt >= 10) {
            orange_load_all_done_flg = true;
            cur_mission_idx++;
          }
        }
        std::cout <<"=======MISSION_LOAD_ORANGE_BRICKS DONE======" << std::endl;

        break;
      }

      case MISSION_MOVE_TO_SEGMENT:
      {
        std::cout <<"=========MISSION_MOVE_TO_SEGMENT START=========" << std::endl;

        move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
        move_srv.request.goal = mission_preset.get_segment_pos_tmp();
        move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
        move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
        move_srv.request.destination_name = SEGMENT;

        if(client_ugv_gps.call(move_srv)) {
          if(move_srv.response.success_or_fail == true) {
            std::cout << "UGV ARRIVED AT SEGMENT SECTION" << std::endl;
          }
          else {
            std::cout << "UGV FAIL TO ARRIVE AT SEGMENT SECTION, TRY AGAIN..." << std::endl;
            break;
          }
        }
        else {
          std::cout << "Failed to call UGV GPS NODE (in charge of DEAGYU) service" << std::endl;
          return -1;
        }

        if(move_srv.response.success_or_fail == true)
          cur_mission_idx++;

        std::cout <<"=========MISSION_MOVE_TO_SEGMENT DONE=========" << std::endl;

        break;
      }

      case MISSION_UNLOAD_ORANGE_BRICKS:
      {
        std::cout <<"=======MISSION_FIND_SEGMENT START======" << std::endl;
        {
          move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
          move_srv.request.goal = mission_preset.get_brick_pos_tmp();
          move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
          move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
          move_srv.request.destination_name = BRICKS;
          move_srv.request.target_color_brick = COLOR_ORANGE;
          move_srv.request.target_action = UGV_ACTION_UNLOAD;

          if(client_ugv_local.call(move_srv))
          {
            if(move_srv.response.success_or_fail == true) {
              std::cout << "UGV ARRIVED IN FRONT OF THE ORANGE BRICKS" << std::endl;
            }
            else {
              std::cout << "UGV FAIL TO ARRIVE IN FRONT OF THE ORANGE BRICKS, TRY AGAIN..." << std::endl;
              break;
            }
          }
          else
          {
            std::cout << "Failed to call UGV LOCAL NODE (in charge of DONGHEE) service" << std::endl;
            return -1;
          }
        }
        std::cout <<"=======MISSION_FIND_SEGMENT DONE======" << std::endl;



        std::cout <<"=====MISSION_UNLOAD_ORANGE_BRICKS START======" << std::endl;
        
        bool unload_left_done_flg = false;
        bool unload_right_done_flg = false;

        // TO DO : Align to segment

        //======================= LEFT container unload
        conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
        conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
        conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
        conveyor_srv.request.value = mission_preset.get_conveyor_left_unload_vel();

        if(client_conveyor.call(conveyor_srv)) {
          std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          if(conveyor_srv.response.comm_result == true) {
            sleep(mission_preset.get_conveyor_defualt_unload_time());
            
            //======================= conveyor stop
            conveyor_srv.request.value = 0;
            if(client_conveyor.call(conveyor_srv)) {
              std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
            }
            else {
              std::cout << "Failed to call service" << std::endl;
              return -1;
            }

            unload_left_done_flg = true;
          }
        }
        else
        {
          std::cout << "Failed to call service" << std::endl;
          return -1;
        }

        //======================= TO DO : Align to segment the other part for unloading

        //======================= RIGHT container unload
        conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
        conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
        conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
        conveyor_srv.request.value = mission_preset.get_conveyor_right_unload_vel();

        if(client_conveyor.call(conveyor_srv)) {
          std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          sleep(mission_preset.get_conveyor_defualt_unload_time());

          //======================= conveyor stop
          conveyor_srv.request.value = 0;
          if(client_conveyor.call(conveyor_srv)) {
            std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          }
          else {
            std::cout << "Failed to call service" << std::endl;
            return -1;
          }

          unload_right_done_flg = true;
        }

        else
        {
          std::cout << "Failed to call service" << std::endl;
          return -1;
        }

        if(unload_left_done_flg && unload_right_done_flg == true) {
          orange_unload_done_flg = true;
          orange_load_all_done_flg = !orange_unload_done_flg;
          orange_load_left_cnt = 0;
          orange_load_right_cnt = 0;
          cur_mission_idx++;
        }

        std::cout <<"=====MISSION_UNLOAD_ORANGE_BRICKS DONE======" << std::endl;

        break;
      }
      
      case MISSION_LOAD_MULTICOLOR_BRICKS:
      {
        std::cout <<"=====MISSION_LOAD_MULTICOLOR_BRICKS START======" << std::endl;

        // TO DO : brick re-arrange function call for each side of the container
        // vector<vector<int>> vect
        // vect[0][:] --> first layer
        // vect[:][0] --> inside of the container
        // re_arrange();

        vector<vector<int>> re_arraged_left{ { 1, 2, 3 }, 
                                             { 1, 2, 3 }, 
                                             { 3, 2, 1 } };
                                             
        vector<vector<int>> re_arraged_right{ { 1, 2, 3 }, 
                                             { 1, 2, 3 }, 
                                             { 3, 2, 1 } };

        //======================= for left container
        mission_preset.set_ur_action(UR_ACTION_LOAD);
        mission_preset.set_target_brick_container_side(CONTAINER_LEFT);
        
        int left_layer_cnt = 0;
        int left_layer_ele_cnt = 0;
        bool left_done_flg = false;

        for(left_layer_cnt = 0; left_layer_cnt < re_arraged_left.size(); ) 
        {
          for(left_layer_ele_cnt = 0; left_layer_ele_cnt < re_arraged_left[left_layer_cnt].size(); ) 
          {
            bool aligned_flg = false;

            std::cout <<"=======MISSION_GO_TO_COLOR_BRICKS START======" << std::endl;
            {
              move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
              move_srv.request.goal = mission_preset.get_brick_pos_tmp();
              move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
              move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
              move_srv.request.destination_name = BRICKS;
              move_srv.request.target_color_brick = re_arraged_left[left_layer_cnt][left_layer_ele_cnt];
              move_srv.request.target_action = UGV_ACTION_LOAD;

              if(client_ugv_local.call(move_srv))
              {
                if(move_srv.response.success_or_fail == true) {
                  std::cout << "UGV ARRIVED IN FRONT OF THE COLOR BRICKS" << std::endl;
                  aligned_flg = true;
                }
                else {
                  std::cout << "UGV FAIL TO ARRIVE IN FRONT OF THE COLOR BRICKS, TRY AGAIN..." << std::endl;
                  aligned_flg = false;
                  break;
                }
              }
              else
              {
                std::cout << "Failed to call UGV LOCAL NODE (in charge of DONGHEE) service" << std::endl;
                return -1;
              }
            }
            std::cout <<"=======MISSION_GO_TO_COLOR_BRICKS DONE======" << std::endl;

            if(aligned_flg == true) {
              mission_preset.set_ur_target_brick_color(re_arraged_left[left_layer_cnt][left_layer_ele_cnt]);

              ur_srv.request.target_node_name = mission_preset.get_ur_target_node_name();
              ur_srv.request.target_load_or_unload = mission_preset.get_ur_action();
              ur_srv.request.target_brick_color_code = mission_preset.get_ur_target_brick_color();
              ur_srv.request.target_brick_container_side_left_right = mission_preset.get_target_brick_container_side();

              if(client_ur.call(ur_srv)) 
              {
                std::cout << "Response : " << ur_srv.response.success_or_fail << std::endl;
                if(ur_srv.response.success_or_fail == true)
                {
                  left_layer_ele_cnt++;
                }
                else
                {
                  // workspace unreachable or not detected target brick
                  break;
                }
              }
              else 
              {
                std::cout << "Failed to call service" << std::endl;
              }
            }
          }
          if(left_layer_ele_cnt >= re_arraged_left[left_layer_cnt].size()){
            left_layer_cnt++;
          }
        }


        //======================= left conveyor move back for brick aligning
        if(left_layer_ele_cnt >= re_arraged_left[left_layer_cnt].size() && left_layer_cnt > re_arraged_left.size()) {
          
          conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
          conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
          conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
          conveyor_srv.request.value = mission_preset.get_conveyor_left_load_vel();

          if(client_conveyor.call(conveyor_srv)) {
            std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
            if(conveyor_srv.response.comm_result == true) {
              sleep(mission_preset.get_conveyor_defualt_move_back_time());
              
              // conveyor stop
              conveyor_srv.request.value = 0;
              if(client_conveyor.call(conveyor_srv)) {
                std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
                left_done_flg = true;
              }
              else {
                std::cout << "Failed to call service" << std::endl;
                return -1;
              }
            }
          }
          else
          {
            std::cout << "Failed to call service" << std::endl;
            return -1;
          }
        }


        

        //======================= for right container
        mission_preset.set_ur_action(UR_ACTION_LOAD);
        mission_preset.set_target_brick_container_side(CONTAINER_RIGHT);

        int right_layer_cnt = 0;
        int right_layer_ele_cnt = 0;
        bool right_done_flg = false;

        for(right_layer_cnt = 0; right_layer_cnt < re_arraged_left.size(); ) 
        {
          for(right_layer_ele_cnt = 0; right_layer_ele_cnt < re_arraged_left[right_layer_cnt].size(); ) 
          {
            bool aligned_flg = false;

            std::cout <<"=======MISSION_GO_TO_COLOR_BRICKS START======" << std::endl;
            {
              move_srv.request.target_node_name = mission_preset.get_move_ugv_target_node_name();
              move_srv.request.goal = mission_preset.get_brick_pos_tmp();
              move_srv.request.move_cmd_weight = mission_preset.get_move_ugv_cmd_vel_weight();
              move_srv.request.delay_before_start_sec = mission_preset.get_move_ugv_before_delay_sec();
              move_srv.request.destination_name = BRICKS;
              move_srv.request.target_color_brick = re_arraged_right[right_layer_cnt][right_layer_ele_cnt];
              move_srv.request.target_action = UGV_ACTION_LOAD;

              if(client_ugv_local.call(move_srv))
              {
                if(move_srv.response.success_or_fail == true) {
                  std::cout << "UGV ARRIVED IN FRONT OF THE COLOR BRICKS" << std::endl;
                  aligned_flg = true;
                }
                else {
                  std::cout << "UGV FAIL TO ARRIVE IN FRONT OF THE COLOR BRICKS, TRY AGAIN..." << std::endl;
                  aligned_flg = false;
                  break;
                }
              }
              else
              {
                std::cout << "Failed to call UGV LOCAL NODE (in charge of DONGHEE) service" << std::endl;
                return -1;
              }
            }
            std::cout <<"=======MISSION_GO_TO_COLOR_BRICKS DONE======" << std::endl;

            if(aligned_flg == true) {
              mission_preset.set_ur_target_brick_color(re_arraged_left[right_layer_cnt][right_layer_ele_cnt]);

              ur_srv.request.target_node_name = mission_preset.get_ur_target_node_name();
              ur_srv.request.target_load_or_unload = mission_preset.get_ur_action();
              ur_srv.request.target_brick_color_code = mission_preset.get_ur_target_brick_color();
              ur_srv.request.target_brick_container_side_left_right = mission_preset.get_target_brick_container_side();

              if(client_ur.call(ur_srv)) 
              {
                std::cout << "Response : " << ur_srv.response.success_or_fail << std::endl;
                if(ur_srv.response.success_or_fail == true)
                {
                  right_layer_ele_cnt++;
                }
                else
                {
                  // workspace unreachable or not detected target brick
                  break;
                }
              }
              else 
              {
                std::cout << "Failed to call service" << std::endl;
              }
            }          
          }
          if(right_layer_ele_cnt >= re_arraged_right[right_layer_cnt].size()){
            right_layer_cnt++;
          }
        }


        //======================= right conveyor move back for brick aligning
        if(right_layer_ele_cnt >= re_arraged_right[right_layer_cnt].size() && right_layer_cnt > re_arraged_right.size()) {

          conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
          conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
          conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
          conveyor_srv.request.value = mission_preset.get_conveyor_right_load_vel();

          if(client_conveyor.call(conveyor_srv)) {
            std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
            if(conveyor_srv.response.comm_result == true) {
              sleep(mission_preset.get_conveyor_defualt_move_back_time());
              
              // conveyor stop
              conveyor_srv.request.value = 0;
              if(client_conveyor.call(conveyor_srv)) {
                std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
                right_done_flg = true;
              }
              else {
                std::cout << "Failed to call service" << std::endl;
                return -1;
              }
            }
          }
          else
          {
            std::cout << "Failed to call service" << std::endl;
            return -1;
          }
        }
        
        if(left_done_flg == true && right_done_flg == true) {
          cur_mission_idx++;
        }
        
        std::cout <<"=====MISSION_LOAD_MULTICOLOR_BRICKS DONE======" << std::endl;

        break;
      }

      case MISSION_UNLOAD_MULTICOLOR_BRICKS:
      {
        std::cout <<"=====MISSION_UNLOAD_MULTICOLOR_BRICKS START=====" << std::endl;

        bool unload_left_done_flg = false;
        bool unload_right_done_flg = false;

        //======================= TO DO : Align to segment

        //======================= LEFT container unload
        conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
        conveyor_srv.request.id = mission_preset.get_conveyor_left_id();
        conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
        conveyor_srv.request.value = mission_preset.get_conveyor_left_unload_vel();

        if(client_conveyor.call(conveyor_srv)) {
          std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          if(conveyor_srv.response.comm_result == true) {
            sleep(mission_preset.get_conveyor_defualt_unload_time());

            //======================= conveyor stop
            conveyor_srv.request.value = 0;
            if(client_conveyor.call(conveyor_srv)) {
              std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
            }
            else {
              std::cout << "Failed to call service" << std::endl;
              return -1;
            }

            unload_left_done_flg = true;
          }
        }
        else
        {
          std::cout << "Failed to call service" << std::endl;
          return -1;
        }

        //======================= TO DO : Align to segment the other part for unloading

        //======================= RIGHT container unload
        conveyor_srv.request.command = mission_preset.get_conveyor_item_cmd();
        conveyor_srv.request.id = mission_preset.get_conveyor_right_id();
        conveyor_srv.request.addr_name = mission_preset.get_conveyor_item_addr();
        conveyor_srv.request.value = mission_preset.get_conveyor_right_unload_vel();

        if(client_conveyor.call(conveyor_srv)) {
          std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          sleep(mission_preset.get_conveyor_defualt_unload_time());

          //======================= conveyor stop
          conveyor_srv.request.value = 0;
          if(client_conveyor.call(conveyor_srv)) {
            std::cout << "Response : " << conveyor_srv.response.comm_result << std::endl;
          }
          else {
            std::cout << "Failed to call service" << std::endl;
            return -1;
          }

          unload_right_done_flg = true;
        }

        else
        {
          std::cout << "Failed to call service" << std::endl;
        }

        if(unload_left_done_flg && unload_right_done_flg == true) {
          multicolor_unload_done_flg = true;
          multicolor_load_done_flg = !multicolor_unload_done_flg;
          blue_load_cnt = 0;
          green_load_cnt = 0;
          red_load_cnt = 0;
          cur_mission_idx++;
        }

        std::cout <<"=====MISSION_UNLOAD_MULTICOLOR_BRICKS DONE=====" << std::endl;

        break;
      }

      case MISSION_FINISH:
      {
        std::cout <<"=============MISSION_FINISH=============" << std::endl;
        state_finish_flg = true;
        break;

      }

      default:
      {
        std::cout <<"STATE ERROR!!" << std::endl;
      }
    }
  }

  ros::spin();

  return 0;
}
