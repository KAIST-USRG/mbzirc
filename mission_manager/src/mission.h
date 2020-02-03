#include <iostream>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

using namespace std;

struct mission_struct{
    int mission = -1;

    // mission type : ready = 0, move = 1, load = 2, unload = 3, path searching = 4, finish = 5 or not init = -1
    int mission_type = -1; 

    // mission state : mission is not yet executed = -1, something wrong = 0, mission is ongoing = 1; mission finish = 2 
    int mission_states = -1;
};

class Mission{
private:
    string m_name_str;
    int m_type_int; // mission type : ready = 0, move = 1, load = 2, unload = 3, path searching = 4, finish = 5 or not init = -1
    int m_state_int; // mission state : mission is not yet executed = -1, something wrong = 0, mission is ongoing = 1; mission finish = 2

public:
    Mission() {
        m_name_str = "";
        m_type_int = -1;
        m_state_int = -1;
    }
    void set_mission_name(string _mission_name) {
        m_name_str = _mission_name;
    }
    void set_mission_type(int _mission_type) {
        m_type_int = _mission_type;
    }
    void set_mission_state(int _mission_state) {
        m_state_int = _mission_state;
    }

    string get_mission_name() {
        return m_name_str;
    }
    int get_mission_type() {
        return m_type_int;
    }
    int get_mission_state() {
        return m_state_int;
    }

};

enum MISSION_SEQUENCE {
    MISSION_READY,
    MISSION_MOVE_TO_BRICK,
    MISSION_LOAD_ORANGE_BRICKS,
    MISSION_MOVE_TO_SEGMENT,
    MISSION_UNLOAD_ORANGE_BRICKS,
    MISSION_LOAD_MULTICOLOR_BRICKS,
    MISSION_UNLOAD_MULTICOLOR_BRICKS,
    MISSION_FINISH
};

enum UR_action {
    UR_ACTION_LOAD = 0,
    UR_ACTION_UNLOAD
};

enum Brick_color {
    COLOR_ORANGE = 1,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_RED
};

enum Container_side {
    CONTAINER_LEFT = 0,
    CONTAINER_RIGHT
};

class Mission_preset{
private:
    // FOR UGV
    geometry_msgs::PoseStamped m_brick_pos_tmp;
    geometry_msgs::PoseStamped m_segment_pos_tmp;
    string m_move_ugv_target_node_name = "move_base";
    double m_move_ugv_cmd_vel_weitht = 1.0;
    double m_move_ugv_before_delay_sec = 1.0;

    // FOR CONVEYOR
    int m_conveyor_left_id = 5;
    int m_conveyor_right_id = 6;
    string m_conveyor_item_cmd = "";
    string m_conveyor_item_addr = "Goal_Velocity"; // position ctrl mode : Goal_Position
    int m_conveyor_left_load_vel = 3000;
    int m_conveyor_left_unload_vel = -1 *m_conveyor_left_load_vel;
    int m_conveyor_right_load_vel = 3000;
    int m_conveyor_right_unload_vel = -1 *m_conveyor_right_load_vel;
    int m_conveyor_defualt_unload_time = 50*1000; // 50 sec
    int m_conveyor_defualt_move_back_time = 10*1000; // 10 sec
    int m_conveyor_running_time = 10*1000; // 10 sec

    // FOR UR - ROBOT ARM
    string m_ur_target_node_name = "ur_control";
    bool m_ur_target_load_or_unload = -1;           // LOAD=0, UNLOAD=1
    int m_ur_target_brick_color_code = -1;          // COLOR_CODE : orange=1, blue=2, green=3, red = 4
    bool m_target_brick_container_side = -1;        // CONTAINER SIDE : LEFT=0, RIGHT=1

    string m_name_str;
    int m_type_int;                                 // mission type : ready = 0, move = 1, load = 2, unload = 3, path searching = 4, finish = 5 or not init = -1
    int m_state_int;                                // mission state : mission is not yet executed = -1, something wrong = 0, mission is ongoing = 1; mission finish = 2

public:
    Mission_preset() {
        m_name_str = "";
        m_type_int = -1;
        m_state_int = -1;

        m_brick_pos_tmp.header.stamp = ros::Time::now();
        m_brick_pos_tmp.header.frame_id = "/odom";
        m_brick_pos_tmp.pose.position.x = 0;
        m_brick_pos_tmp.pose.position.y = 0;
        m_brick_pos_tmp.pose.position.z = 0;
        m_brick_pos_tmp.pose.orientation.x = 0;
        m_brick_pos_tmp.pose.orientation.y = 0;
        m_brick_pos_tmp.pose.orientation.z = 0;
        m_brick_pos_tmp.pose.orientation.w = 0;

        m_segment_pos_tmp.header.stamp = ros::Time::now();
        m_segment_pos_tmp.header.frame_id = "/odom";
        m_segment_pos_tmp.pose.position.x = 10;
        m_segment_pos_tmp.pose.position.y = -5;
        m_segment_pos_tmp.pose.position.z = 0;
        m_segment_pos_tmp.pose.orientation.x = 0;
        m_segment_pos_tmp.pose.orientation.y = 0;
        m_segment_pos_tmp.pose.orientation.z = 0;
        m_segment_pos_tmp.pose.orientation.w = 0;
        
    }
    void set_mission_name(string _mission_name) {
        m_name_str = _mission_name;
    }
    void set_mission_type(int _mission_type) {
        m_type_int = _mission_type;
    }
    void set_mission_state(int _mission_state) {
        m_state_int = _mission_state;
    }

    string get_mission_name() {
        return m_name_str;
    }
    int get_mission_type() {
        return m_type_int;
    }
    int get_mission_state() {
        return m_state_int;
    }

    geometry_msgs::PoseStamped get_brick_pos_tmp() {
        return m_brick_pos_tmp;
    }

    geometry_msgs::PoseStamped get_segment_pos_tmp() {
        return m_segment_pos_tmp;
    }

    double get_move_ugv_cmd_vel_weight() {
        return m_move_ugv_cmd_vel_weitht;
    }

    double get_move_ugv_before_delay_sec() {
        return m_move_ugv_before_delay_sec;
    }

    string get_move_ugv_target_node_name() {
        return m_move_ugv_target_node_name;
    }

    int get_conveyor_left_id() {
        return m_conveyor_left_id;
    }
    int get_conveyor_right_id() {
        return m_conveyor_right_id;
    }

    string get_conveyor_item_cmd() {
        return m_conveyor_item_cmd;
    }

    string get_conveyor_item_addr() {
        return m_conveyor_item_addr;
    }

    int get_conveyor_left_load_vel() {
        return m_conveyor_left_load_vel;
    }

    int get_conveyor_left_unload_vel() {
        return m_conveyor_left_unload_vel;
    }

    int get_conveyor_right_load_vel() {
        return m_conveyor_right_load_vel;
    }

    int get_conveyor_right_unload_vel() {
        return m_conveyor_right_unload_vel;
    }

    void set_conveyor_running_time(int _time_in_sec) {
        m_conveyor_running_time = 1000*_time_in_sec;
    }

    int get_conveyor_running_time() {
        return m_conveyor_running_time;
    }

    int get_conveyor_defualt_unload_time() {
        return m_conveyor_defualt_unload_time;
    }

    int get_conveyor_defualt_move_back_time() {
        return m_conveyor_defualt_move_back_time;
    }

    string get_ur_target_node_name() {
        return m_ur_target_node_name;
    }

    void set_ur_action(int _action) {
        m_ur_target_load_or_unload = _action;
    }

    int get_ur_action() {
        if(m_ur_target_load_or_unload != -1)
            return m_ur_target_load_or_unload;
        else {
            cout << "UR ACTION IS NOT DEFINED... RETURN -1" << endl;
            return -1;
        }
    }

    void set_ur_target_brick_color(int _color) {
        m_ur_target_brick_color_code = _color;
    }

    int get_ur_target_brick_color() {
        if(m_ur_target_brick_color_code != -1)
            return m_ur_target_brick_color_code;
        else {
            cout << "TARGET BRICK COLOR IS NOT DEFINED... RETURN -1" << endl;
            return -1;
        }
    }

    void set_target_brick_container_side(int _side) {
        m_target_brick_container_side = _side;
    }

    int get_target_brick_container_side() {
        if(m_target_brick_container_side != -1)
            return m_target_brick_container_side;
        else {
            cout << "CONTAINER SIDE IS NOT DEFINED... RETURN -1" << endl;
            return -1;
        }
    }


};
