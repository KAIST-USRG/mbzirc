/* IMPORTANT
*  reference frame (top view)
*
*  1) Camera frame
*     - X: East
*     - Y: South
*     - Z: Down
*
*  2) UGV frame
*     - X: East
*     - Y: North
*     - Z: Up
*/
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// Move it
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// srv
#include "mbzirc_msgs/ur_move.h"
#include "mbzirc_msgs/go_to_brick.h"
#include "mbzirc_msgs/place_in_container.h"
#include "mbzirc_msgs/visual_servo_XY.h"
#include "mbzirc_msgs/visual_servo_yaw.h"

// msg
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Range.h>

#include <tf/transform_broadcaster.h>


#define DEBUG
#define DELAY                 1.0         // for sleep function => robot updating states => 0.4 s fail (?)
#define PLANNING_TIMEOUT      2
#define NUM_SUM               3           // to average the pose message
#define NUM_DISCARD           100
#define DIST_EE_TO_MAGNET     0.077
#define DIST_CAM_TO_EE        -0.015
#define DIST_LIDAR_TO_MAGNET  0.06
#define Z_OFFSET              0.07
#define BELT_BASE_HEIGHT      0.26

// color code
#define RED                   0
#define GREEN                 1
#define BLUE                  2
#define ORANGE                3

// side code
#define LEFT                  0
#define RIGHT                 1


namespace rvt = rviz_visual_tools;

// for rotation of objects
tf2::Quaternion q;

// FLAG for robot motion

bool FLAG_READ_CAM_DATA   = false;    // Should the arm read message from camera?
bool FLAG_SWITCH_TOUCHED  = false;
bool FLAG_YAW_INCORRECT   = false;
bool FLAG_XY_STOP         = false;
bool FLAG_YAW_STOP        = false;


int   gb_count_pose_msg   = 0;
int   gb_discard_noise    = 0;
int   gb_count_move       = 0;  // 0 for moveXY, 1 for moveZ
int   gb_count_box        = 0;
int   gb_servo_stop_signal= 0;
float gb_x_sum            = 0;
float gb_y_sum            = 0;
float gb_z_sum            = 0;
float gb_xq_sum           = 0;
float gb_yq_sum           = 0;
float gb_zq_sum           = 0;
float gb_wq_sum           = 0;


class Arm{
private:
  // Initialization
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit_visual_tools::MoveItVisualTools visual_tools;
  moveit::core::RobotStatePtr current_state;
  const robot_state::JointModelGroup* joint_model_group;
  Eigen::Isometry3d text_pose;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  moveit_msgs::CollisionObject brick; // Define a collision object ROS message.

  // for Cartesian Path
  const double jump_threshold = 0.0;       // read only.
  const double eef_step       = 0.01;      // read only.
  const double PI             = 3.141592653589793;
  double       fraction       = 0.0;


  // Publisher

  ros::Publisher avg_pose_pub;
  ros::Publisher magnet_state_pub;  
  ros::Publisher moveToDefault_finished_flag_pub;
  ros::Publisher readCamData_flag_pub;
  

  // Subscriber
  ros::Subscriber manual_moveXVZ_sub;
  ros::Subscriber moveToDefault_flag_sub;
  ros::Subscriber moveToStorageSide_flag_sub;
  ros::Subscriber plate_yaw_incorrect_sub;
  ros::Subscriber pose_from_cam_sub;
  ros::Subscriber readCamData_flag_sub;
  ros::Subscriber servo_stop_sub;
  ros::Subscriber switch_state_sub;


  // msg
  std_msgs::Bool finish_stop_xy_msg;
  std_msgs::Bool finish_stop_yaw_msg;
  std_msgs::Bool magnet_state_msg;
  std_msgs::Bool moveToDefault_finished_flag_msg;
  std_msgs::Bool moveToDefault_flag_msg;
  std_msgs::Bool moveToStorageSide_finished_flag_msg;
  std_msgs::Bool moveToStorageSide_flag_msg;
  std_msgs::Bool readCamData_flag_msg;

  std_msgs::Int16 box_count_msg;
  
  geometry_msgs::Pose target_pose;
  geometry_msgs::Pose avg_pose_msg; // absolute position w.r.t UR5 base
  geometry_msgs::Point avg_Z_msg;

  // ServiceClient
  ros::ServiceClient go_to_brick_sc;
  ros::ServiceClient place_in_container_sc;
  ros::ServiceClient visual_servo_XY_sc;
  ros::ServiceClient visual_servo_yaw_sc;

  // ServiceServer
  ros::ServiceServer go_to_brick_ss;
  ros::ServiceServer place_in_container_ss;
  ros::ServiceServer ur_move_ss;          // request is from mission_manager node


  // Service
  mbzirc_msgs::go_to_brick go_to_brick_srv;
  mbzirc_msgs::place_in_container place_in_container_srv;
  mbzirc_msgs::visual_servo_XY visual_servo_XY_srv;
  mbzirc_msgs::visual_servo_yaw visual_servo_yaw_srv;

  // Node Handle
  // ros::NodeHandle node_handle;
  ros::NodeHandle nh;


public:

  Arm():nh("~"), move_group("manipulator"), visual_tools("base_link") {

    // ========================  Initialization
    const std::string PLANNING_GROUP = "manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We can now setup the PlanningPipeline
    // object, which will use the ROS parameter server
    // to determine the set of request adapters and the
    // planning plug in to use
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, nh, "planning_plugin", "request_adapters"));

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    current_state = move_group.getCurrentState();

    robot_trajectory::RobotTrajectory rt(robot_model_loader.getModel(), PLANNING_GROUP);

    joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    text_pose = Eigen::Isometry3d::Identity();
    initWall();

    // ========================  Node Communications
    // Publisher
    avg_pose_pub  = nh.advertise<geometry_msgs::Pose>("/avg_pose", 10);
    magnet_state_pub = nh.advertise<std_msgs::Bool>("/magnet_on", 10);
    moveToDefault_finished_flag_pub = nh.advertise<std_msgs::Bool>("/moveToDefault_finish_flag", 10);
    readCamData_flag_pub = nh.advertise<std_msgs::Bool>("/readCamData_Flag", 10);


    // Subscriber
    
    manual_moveXVZ_sub = nh.subscribe("/manual_moveXVZ", 10, &Arm::manual_moveXYZ, this);
    moveToDefault_flag_sub = nh.subscribe("/moveToDefault_flag", 10, &Arm::moveToDafaultFlagCallback, this);
    plate_yaw_incorrect_sub = nh.subscribe("/ur5_rotation_incorrect", 1, &Arm::plateYawIncorrectCallback, this);
    pose_from_cam_sub = nh.subscribe("/teraranger_evo", 10, &Arm::calcAvgCallback_Z, this);
    readCamData_flag_sub = nh.subscribe("/readCamData_Flag", 100, &Arm::readCamDataFlagCallback, this);
    servo_stop_sub = nh.subscribe("/ur5_stop", 1, &Arm::servoStopCallback, this);
    switch_state_sub = nh.subscribe("/switch_state", 1, &Arm::switchStateCallback, this);
    

    // Service Client
    go_to_brick_sc = nh.serviceClient<mbzirc_msgs::go_to_brick>("/go_to_brick");
    place_in_container_sc = nh.serviceClient<mbzirc_msgs::place_in_container>("/place_in_container");
    visual_servo_XY_sc = nh.serviceClient<mbzirc_msgs::visual_servo_XY>("/visual_servo_XY");
    visual_servo_yaw_sc = nh.serviceClient<mbzirc_msgs::visual_servo_yaw>("/visual_servo_yaw");


    // Service Server
    go_to_brick_ss = nh.advertiseService("/go_to_brick", &Arm::_goToBrickServiceCallback, this);
    place_in_container_ss = nh.advertiseService("/place_in_container", &Arm::_placeInContainerServiceCallback, this);
    ur_move_ss = nh.advertiseService("ur_move", &Arm::urMoveCallback, this);


    ros::Duration(1).sleep();
    // Turn on Magnet
    magnet_state_msg.data = true;
    magnet_state_pub.publish(magnet_state_msg);
    ROS_INFO("Initialize: MAGNET_ON");

    // DEBUG
    moveToStorageSide_flag_sub = nh.subscribe("/moveToStorageSide_flag", 10, &Arm::moveToStorageSideFlagCallback, this);
  }

  // ==================== Service Callback Function ==================== //
  bool urMoveCallback(mbzirc_msgs::ur_move::Request  &req,
                       mbzirc_msgs::ur_move::Response  &res)
  {    
    /*  The service server to communicate with mission_manager (central planner)
    */

    ROS_INFO("========== Adjust Default Position According to the target container side ==========");
    {
      moveToDefault(RIGHT);

      // Need the EE to be parallel to the ground
      current_state = move_group.getCurrentState();
      ros::Duration(0.5).sleep();
      current_state = move_group.getCurrentState();
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      joint_group_positions[0] = PI;  // Radian
      joint_group_positions[1] = -PI/2;
      joint_group_positions[2] = PI/180*60;
      joint_group_positions[3] = 2 * PI - PI/180 * 60;
      joint_group_positions[4] = -PI/2;
      joint_group_positions[5] = 0;
      move_group.setJointValueTarget(joint_group_positions);
      move_group.setMaxVelocityScalingFactor(0.01);
      move_group.setMaxAccelerationScalingFactor(0.01);
      move_group.setPlanningTime(PLANNING_TIMEOUT);
      move_group.setGoalJointTolerance(0.00000001);


      bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
      ROS_INFO_NAMED("tutorial", "Make EE parallel to the ground: %s", success ? "SUCCEEDED" : "FAILED");

      // #ifdef DEBUG
        // visual_tools.prompt("Press 'next' to make EE parallel to the ground");
      // #endif

      move_group.move();                      // BLOCKING FUNCTION

    }
    
    
    ROS_INFO("============== SERVICE CLIENT to Camera node: Visual Servo XY ==============");
    {
      // reset flag used in this Service
      gb_servo_stop_signal = 0;
      FLAG_XY_STOP = false;
      visual_servo_XY_srv.request.waiting_for_data_XY = true;
      visual_servo_XY_srv.request.brick_color_code = req.target_brick_color_code;
      if(visual_servo_XY_sc.call(visual_servo_XY_srv))
      {
        std::cout << "visual_servo_XY_sc: Response : " << visual_servo_XY_srv.response.x << std::endl;
        std::cout << "visual_servo_XY_sc: Response : " << visual_servo_XY_srv.response.y << std::endl;
        float MAX_DIST = 1;
        bool xy_success = moveXYZSlowly(visual_servo_XY_srv.response.x * MAX_DIST, 
                                       visual_servo_XY_srv.response.y * MAX_DIST, 
                                       0, 0.02, 0.02);

        if (FLAG_XY_STOP != true) // cannot reach to the brick
        {
          ROS_INFO("Visual Servo XY: Cannot reach the brick");
          res.workspace_reachable = false;
          res.success_or_fail = false;   
          return true;
        }

      }else 
      {  // fail to request service
        std::cout << "visual_servo_XY_sc: Failed to call service" << std::endl;
        return false;
      }
    }

    
    ROS_INFO("================== Visual Servo yaw  ==================");
    {
      // reset flag used in this Service
      FLAG_YAW_STOP = false;
      visual_servo_yaw_srv.request.waiting_for_data_yaw = true;
      visual_servo_yaw_srv.request.increase_margin = false;
      while(true)
      {
        if(visual_servo_yaw_sc.call(visual_servo_yaw_srv))
        {
          if (FLAG_YAW_STOP == true)
          {
            std::cout << "No yaw needed"<< std::endl;
            break;
          }

          std::cout << "visual_servo_yaw_sc: Response : " << visual_servo_yaw_srv.response.yaw << std::endl;
          moveYawSlowly(LEFT, 0.05, 0.05);
          if(FLAG_YAW_INCORRECT == true)
          {
            resetYaw();
            moveYawSlowly(RIGHT, 0.05, 0.05);
          }

          if (FLAG_YAW_STOP == true) // cannot reach to the brick
          {
            break;
          }else
          {
            visual_servo_yaw_srv.request.waiting_for_data_yaw = true;
            visual_servo_yaw_srv.request.increase_margin = true;
            resetYaw();
          }
          

        }else 
        {  // fail to request service
          std::cout << "visual_servo_yaw_sc: Failed to call service" << std::endl;
          return false;
        }
      }
      
    }


    ROS_INFO("====================== Trigger to read data from camera ======================");
    {
      readCamData_flag_msg.data = true;
      readCamData_flag_pub.publish(readCamData_flag_msg);
    }

    ROS_INFO("====================== SERVICE CLIENT internal: grab the brick ======================");
    {
      //  ====================== getting the brick  ====================== //
      //  1) check the workspace_reachable
      //  2) go to get the brick
      //  3) go back to default position
      geometry_msgs::PoseConstPtr brick_pose_msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/avg_pose");
      ROS_INFO("brick_pose_msg: x = %lf, y = %lf, z = %lf", brick_pose_msg->position.x, 
                                                            brick_pose_msg->position.y, 
                                                            brick_pose_msg->position.z);
      ROS_INFO("brick_pose_msg: qx = %lf, qy = %lf, qz = %lf, qw = %lf", brick_pose_msg->orientation.x, 
                                                                          brick_pose_msg->orientation.y, 
                                                                          brick_pose_msg->orientation.z,
                                                                          brick_pose_msg->orientation.w);
      go_to_brick_srv.request.brick_color_code = req.target_brick_color_code;
      go_to_brick_srv.request.container_side = req.target_brick_container_side_left_right;
      go_to_brick_srv.request.x = brick_pose_msg->position.x;
      go_to_brick_srv.request.y = brick_pose_msg->position.y;
      go_to_brick_srv.request.z = brick_pose_msg->position.z;
      go_to_brick_srv.request.qx = brick_pose_msg->orientation.x;
      go_to_brick_srv.request.qy = brick_pose_msg->orientation.y;
      go_to_brick_srv.request.qz = brick_pose_msg->orientation.z;
      go_to_brick_srv.request.qw = brick_pose_msg->orientation.w;
      
      if(go_to_brick_sc.call(go_to_brick_srv)) 
      {
        std::cout << "go_to_brick_sc: Response : " << go_to_brick_srv.response.success << std::endl;
        if (!go_to_brick_srv.response.workspace_reachable) // Check if that position is in workspace
        {
          // Return to mission maneger
          ROS_INFO("go_to_brick_sc: Cannot find the correct Yaw angle");
          res.workspace_reachable = false; 
          return true;
        }

        if (go_to_brick_srv.response.success)
          gb_count_box =  (gb_count_box % 5) + 1;
      }
      else {  // fail to request service
        std::cout << "go_to_brick_sc: Failed to call service" << std::endl;
        return false;
      }
      ROS_INFO("Finish go_to_brick service");
    }

    //  ====================== Control Convey Belt ====================== //
    // DO SOMETHING

    ROS_INFO("====================== SERVICE CLIENT internal: put on the container ======================");
    {
      place_in_container_srv.request.order_of_this_brick = gb_count_box;
      place_in_container_srv.request.brick_container_side = req.target_brick_container_side_left_right;

      // service call
      if(place_in_container_sc.call(place_in_container_srv)) 
      {
        std::cout << "place_in_container_sc: Response : " << place_in_container_srv.response.success << std::endl;
      }
      else {  // fail to request service
        std::cout << "place_in_container_sc: Failed to call service" << std::endl;
        return false;
      }
    }

    // All the services proceeded correctly
    res.workspace_reachable = true;
    res.success_or_fail = true;  
    return true;
  }

  // ==================== Internal Service Callback Function ==================== //
  bool _goToBrickServiceCallback(mbzirc_msgs::go_to_brick::Request  &req,
                        mbzirc_msgs::go_to_brick::Response  &res)
  {
    /*  This function moves UR5 arm to get the brick
    *   1) Align the end effector yaw angle with the brick
    *   2) Move close to the brick
    *   3) Slowly moving down until the switch is triggered
    *   4) Open the magnet => this make the force stronger
    *   5) Go back to default position
    */

    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //  ====================== Move close to the brick  ====================== //
    {
      // moving +toX => +X in UGV frame
      // moving +toY => -Y in UGV frame
      // moving +toZ => -Z in UGV frame

      std::vector<geometry_msgs::Pose> waypoints_down;
      target_pose = move_group.getCurrentPose().pose; 
      ros::Duration(0.5).sleep();
      target_pose = move_group.getCurrentPose().pose;

      // first way point
      // target_pose.position.x = target_pose.position.x - 0.03;
      // target_pose.position.y = target_pose.position.y + 0.03;
      // target_pose.position.z = target_pose.position.z - (req.z / 2);
      // waypoints_down.push_back(target_pose);

      // second waypoint (target)
      // target_pose.position.x = target_pose.position.x + 0.03;
      // target_pose.position.y = target_pose.position.y - 0.03;
      // target_pose.position.z = target_pose.position.z - (req.z / 2) + DIST_LIDAR_TO_MAGNET + Z_OFFSET;
      target_pose.position.z = 0.10;
      waypoints_down.push_back(target_pose);

      move_group.setPlanningTime(PLANNING_TIMEOUT);
      

      moveit_msgs::RobotTrajectory trajectory_down;

      for(int i = 0; i < 3; i++)
      {
        fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
        if (fraction == 1.0)
          break;
      }
      
      if(fraction < 1.0)  // Don't proceed if cannot reach the position
      {
        res.workspace_reachable = false;
        res.success = false;
        return true;
      }

      // ================================= modify the velocity of Cartesian path ================================= //
      // First create a RobotTrajectory object
      robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_down);
      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      // Fourth compute computeTimeStamps
      bool success = iptp.computeTimeStamps(rt, 0.2, 0.2);
      rt.getRobotTrajectoryMsg(trajectory_down);
      ROS_INFO_NAMED("tutorial", "_goToBrickServiceCallback: Step 2: Move close to the brick (%.2f%% achieved)", fraction * 100.0);

      cartesian_plan.trajectory_ = trajectory_down;

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishPath(waypoints_down, rvt::LIME_GREEN, rvt::SMALL);
      for (std::size_t i = 0; i < waypoints_down.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints_down[i], "pt" + std::to_string(i), rvt::SMALL);
      visual_tools.trigger();

      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to go down");
      #endif

      move_group.execute(cartesian_plan);
    }
    

    //  ====================== Slowly moving the end-effector down, till it triggered  ====================== //
    while (true)
    {
      ROS_INFO("_goToBrickServiceCallback: Moving Down Slowly");
      moveXYZSlowly(0, 0, -0.50, 0.04, 0.04); // continue to move down until the tactile sensor is triggered

      if (FLAG_SWITCH_TOUCHED == true) // tactile sensor is triggered => stop
      {
        break;
      }
    }

    FLAG_READ_CAM_DATA = false; // Disable reading box pose data stream
    gb_count_move = 0;

    //  ====================== Open magnet to make it stronger  ====================== //
    {
      magnet_state_msg.data = true;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET ON
      ROS_INFO("_goToBrickServiceCallback: MAGNET_ON");
      ros::Duration(1.0).sleep();
      attachBrick(req.brick_color_code);
    }

    

    //  ====================== Move the robot arm back to the higher than default position  ====================== //
    {
      current_state = move_group.getCurrentState();
      ros::Duration(0.5).sleep();
      current_state = move_group.getCurrentState();
      // Next get the current set of joint values for the group.
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = PI;  // Radian
      joint_group_positions[1] = -PI/2;
      joint_group_positions[2] = PI/180 * 45;
      joint_group_positions[3] = 2 * PI - PI/180 * 45;
      joint_group_positions[4] = -PI/2;
      joint_group_positions[5] = 0;
      
      move_group.setJointValueTarget(joint_group_positions);
      move_group.setMaxVelocityScalingFactor(0.4);
      move_group.setMaxAccelerationScalingFactor(0.4);
      move_group.setPlanningTime(PLANNING_TIMEOUT);
      move_group.setGoalJointTolerance(0.01);


      bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
      ROS_INFO_NAMED("tutorial", "Higher moveToDefault: %s", success ? "SUCCEEDED" : "FAILED");

      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to front position");
      #endif

      move_group.move();                      // BLOCKING FUNCTION

      res.success = true;
      res.workspace_reachable = true;
      return true;
    }
    
  }


  bool _placeInContainerServiceCallback(mbzirc_msgs::place_in_container::Request  &req,
                                        mbzirc_msgs::place_in_container::Response  &res)
  {
    /*  This function place brick in one of the container
    *   1) Turn the robot to the target side of the container
    *   2) Move the end effector to the container and move down 
    *   3) Turn off the magnet to drop the brick
    *   4) Go back to the default position
    */
    // Get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //  ====================== Turn the robot arm to the correct container side  ====================== //
    {
      current_state = move_group.getCurrentState();
      ros::Duration(0.5).sleep();
      current_state = move_group.getCurrentState();
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 3 * PI/2; // Turn UR toward the container LEFT side

      move_group.setJointValueTarget(joint_group_positions);
      move_group.setPlanningTime(PLANNING_TIMEOUT);
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxVelocityScalingFactor(0.3);
      move_group.setMaxAccelerationScalingFactor(0.3);

      bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
      ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 2: Turn %s", success ? "SUCCEEDED" : "FAILED");


      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to front position");
      #endif

      move_group.move();                      // BLOCKING FUNCTION
    }

    //  ====================== go to the container  ====================== //
    {
      std::vector<geometry_msgs::Pose> waypoints_to_storage;
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
      ros::Duration(0.5).sleep();
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

      if (req.brick_container_side == LEFT)
      {
        target_pose.position.x = -0.565;
        target_pose.position.y = -0.1;
        target_pose.position.z = 0.65;
      }else
      {
        target_pose.position.x = 0.565;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.65;
      }

      waypoints_to_storage.push_back(target_pose);
      
      target_pose.position.z = (req.order_of_this_brick  - 1) * 0.22;
      ROS_INFO("z = %lf", target_pose.position.z);

      waypoints_to_storage.push_back(target_pose);

      move_group.setMaxVelocityScalingFactor(0.2);
      move_group.setMaxAccelerationScalingFactor(0.2);
      move_group.setPlanningTime(PLANNING_TIMEOUT);

      moveit_msgs::RobotTrajectory trajectory_to_storage;
      for(int i = 0; i < 3; i++)
      {
        fraction = move_group.computeCartesianPath(waypoints_to_storage, eef_step, jump_threshold, trajectory_to_storage);
        if (fraction == 1.0)
          break;
      }
      // ================================= modify the velocity of Cartesian path ================================= //
      // First create a RobotTrajectory object
      robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_to_storage);
      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      // Fourth compute computeTimeStamps
      bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);
      rt.getRobotTrajectoryMsg(trajectory_to_storage);

      ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 4: Cartesian To Storage (%.2f%% achieved)", fraction * 100.0);
      cartesian_plan.trajectory_ = trajectory_to_storage;

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishPath(waypoints_to_storage, rvt::LIME_GREEN, rvt::SMALL);
      for (std::size_t i = 0; i < waypoints_to_storage.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints_to_storage[i], "pt" + std::to_string(i), rvt::SMALL);
      visual_tools.trigger();

      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to go down");
      #endif

      move_group.execute(cartesian_plan);
    }

    //  ====================== turn off the magnet to detach the brick  ====================== //
    {
      magnet_state_msg.data = false;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET OFF
      ROS_INFO("_placeInContainerServiceCallback: MAGNET_OFF");
      ros::Duration(5).sleep();
      
      detachBrick();
      deleteObject();
    }

    //  ====================== move back to the default position  ====================== //
    {
      if (req.order_of_this_brick == 1 || req.order_of_this_brick == 2)
      {
        std::vector<geometry_msgs::Pose> waypoints_up_from_storage;
        target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
        ros::Duration(0.5).sleep();
        target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
        target_pose.position.z = 0.30;
        waypoints_up_from_storage.push_back(target_pose);
        move_group.execute(cartesian_plan);
        move_group.setPlanningTime(PLANNING_TIMEOUT);
        moveit_msgs::RobotTrajectory trajectory_up_from_storage;
        for(int i = 0; i < 3; i++)
        {
          fraction = move_group.computeCartesianPath(waypoints_up_from_storage, eef_step, jump_threshold, trajectory_up_from_storage);
          if (fraction == 1.0)
            break;
        }
        ROS_INFO_NAMED("tutorial", "trajectory_up_from_storage: Cartesian To Storage (%.2f%% achieved)", fraction * 100.0);
        cartesian_plan.trajectory_ = trajectory_up_from_storage;

        move_group.execute(cartesian_plan);
      }
      

      moveToDefault(RIGHT);
      
      moveToDefault_finished_flag_msg.data = true;
      moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
    }

    //  ====================== turn the magnet on again  ====================== //
    {
      magnet_state_msg.data = true;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET OFF
      ROS_INFO("_placeInContainerServiceCallback: MAGNET_ON");
      return true;
      
    }
  }

  // ==================== Topic Callback Function ==================== //
  void calcAvgCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
    /*  Read the brick pose from the camera
    *   publish the processed data
    *   The data is expressed in Camera Frame
    */
    if (FLAG_READ_CAM_DATA == true){
      if (gb_count_pose_msg < NUM_SUM + NUM_DISCARD){

        if (gb_count_pose_msg <= NUM_DISCARD){  // discard the initial steam data => garbage data
          gb_x_sum = 0;
          gb_y_sum = 0;
          gb_z_sum = 0;
          gb_xq_sum = 0;
          gb_yq_sum = 0;
          gb_zq_sum = 0;
          gb_wq_sum = 0;
        }

        ROS_INFO("calcAvgCallback: Accumulate the data");
        gb_x_sum += msg->position.x;
        gb_y_sum += msg->position.y;
        gb_z_sum += msg->position.z;
        gb_xq_sum += msg->orientation.x;
        gb_yq_sum += msg->orientation.y;
        gb_zq_sum += msg->orientation.z;
        gb_wq_sum += msg->orientation.w;

        gb_count_pose_msg += 1;

      }else{ // picking up should start from DEFAULT position
        int n = NUM_SUM;

        avg_pose_msg.position.x = gb_x_sum/n;
        avg_pose_msg.position.y = gb_y_sum/n;
        avg_pose_msg.position.z = gb_z_sum/n;
        avg_pose_msg.orientation.x = gb_xq_sum/n;
        avg_pose_msg.orientation.y = gb_yq_sum/n;
        avg_pose_msg.orientation.z = gb_zq_sum/n;
        avg_pose_msg.orientation.w = gb_wq_sum/n;
        ROS_INFO("calcAvgCallback: x = %lf, y = %lf, z = %lf", avg_pose_msg.position.x, avg_pose_msg.position.y, avg_pose_msg.position.z);

        avg_pose_pub.publish(avg_pose_msg);

        FLAG_READ_CAM_DATA = false;

        gb_x_sum = 0;
        gb_y_sum = 0;
        gb_z_sum = 0;
        gb_xq_sum = 0;
        gb_yq_sum = 0;
        gb_zq_sum = 0;
        gb_wq_sum = 0;

        gb_count_pose_msg = 0; // reset the counter of data to be averaged


      }
    }else{
      // DON'T get the stream data from camera and clear garbage data
      gb_x_sum = 0;
      gb_y_sum = 0;
      gb_z_sum = 0;
      gb_xq_sum = 0;
      gb_yq_sum = 0;
      gb_zq_sum = 0;
      gb_wq_sum = 0;

    }
  }


  void calcAvgCallback_Z(const sensor_msgs::Range::ConstPtr& msg)
  {
    /*  Read the brick pose from the camera
    *   publish the processed data
    *   The data is expressed in Camera Frame
    */
    if (FLAG_READ_CAM_DATA == true){
      if (gb_count_pose_msg < NUM_SUM + NUM_DISCARD){

        if (gb_count_pose_msg <= NUM_DISCARD){  // discard the initial steam data => garbage data
          gb_x_sum = 0;
          gb_y_sum = 0;
          gb_z_sum = 0;
          gb_xq_sum = 0;
          gb_yq_sum = 0;
          gb_zq_sum = 0;
          gb_wq_sum = 0;
        }

        ROS_INFO("calcAvgCallback: Accumulate the data");
        // gb_x_sum += msg->position.x;
        // gb_y_sum += msg->position.y;
        if(msg->range > 1.085)
        {
          gb_z_sum += 1.085;
        }else
        {
          gb_z_sum += msg->range;
        }
        gb_count_pose_msg += 1;

      }else{ // picking up should start from DEFAULT position
        // int n = NUM_SUM;
        
        avg_pose_msg.position.x = gb_x_sum/NUM_SUM;
        avg_pose_msg.position.y = gb_y_sum/NUM_SUM;
        avg_pose_msg.position.z = gb_z_sum/NUM_SUM;
        
        
        avg_pose_msg.orientation.x = gb_xq_sum/NUM_SUM;
        avg_pose_msg.orientation.y = gb_yq_sum/NUM_SUM;
        avg_pose_msg.orientation.z = gb_zq_sum/NUM_SUM;
        avg_pose_msg.orientation.w = gb_wq_sum/NUM_SUM;
        ROS_INFO("calcAvgCallback: x = %lf, y = %lf, z = %lf", avg_pose_msg.position.x, avg_pose_msg.position.y, avg_pose_msg.position.z);

        avg_pose_pub.publish(avg_pose_msg);

        FLAG_READ_CAM_DATA = false;

        gb_x_sum = 0;
        gb_y_sum = 0;
        gb_z_sum = 0;
        gb_xq_sum = 0;
        gb_yq_sum = 0;
        gb_zq_sum = 0;
        gb_wq_sum = 0;

        gb_count_pose_msg = 0; // reset the counter of data to be averaged


      }
    }else{
      // DON'T get the stream data from camera and clear garbage data
      gb_x_sum = 0;
      gb_y_sum = 0;
      gb_z_sum = 0;
      gb_xq_sum = 0;
      gb_yq_sum = 0;
      gb_zq_sum = 0;
      gb_wq_sum = 0;

    }
  }

  void moveToDafaultFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    // Subscribe: moveToDefault_flag
    // Publish: moveToDefault_finish_flag
    if (msg->data == true){
      moveToDefault(LEFT);
      // moveToDefault_finished_flag_msg.data = true;
      // moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
    }else{
      moveToDefault(RIGHT);
      // moveToDefault_finished_flag_msg.data = true;
      // moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
    }
  }


  void plateYawIncorrectCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    /*  The bounding box of the plate is similar when yaw angle is at -rad and rad
    *   We initially turn Left, if it's wrong direction
    *   We turn the otherside (rely on topic msg from camera node)
    */
    if(msg->data == true)
    {
      ROS_INFO("plateYawIncorrectCallback: wrong direction of yaw");
      FLAG_YAW_INCORRECT = true;
      move_group.stop();
    
    }
    else
    {
      // do nothing
    }
  }


  void readCamDataFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    /*  This allows UR5 to take the data stream from camera
    *   data stream is geometry_msgs::Pose of the brick
    */
    if (msg->data == true){
      FLAG_READ_CAM_DATA = true;
    }else{
      FLAG_READ_CAM_DATA = false;
    }
  }


  void servoStopCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    /*  Subscribe to /stop topic msg from camera node
    *   To stop the arm movement in visual servo control
    */
    if (msg->data == true)
    {
      ROS_INFO("servoStopCallback: stop motion");
      move_group.stop();
      if (gb_servo_stop_signal == 0)  // first stop from cam node is XY
      {
        ROS_INFO("XXXXXXXXXXXXXXXX set flag stop xy to true");
        FLAG_XY_STOP = true;
      }else if (gb_servo_stop_signal == 1)
      {
        ROS_INFO("XXXXXXXXXXXXXXXX set flag stop yaw to true");
        FLAG_YAW_STOP = true;
      }else{
        // cannot reach here
        assert(true);
      }
      gb_servo_stop_signal += 1;
    }
    else
    {
      // do nothing
    }
  }


  void switchStateCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    /*  switch always publish 1 if untouched
    *   if either of the switch is touched => publish 0 => the robot arm's motion should stop
    */

    if (msg->data == 1 && FLAG_SWITCH_TOUCHED == false)
    { // The active motion plan should stop when the switch is triggered
      FLAG_SWITCH_TOUCHED = true;
      move_group.stop();
    }
    else if (msg->data == 0)
    {
      FLAG_SWITCH_TOUCHED = false;  // switch is deactivated

    }else
    {
      // do nothing
    }
    
  }

  // ==================== Class Function ==================== //
  void attachBrick(int color)
  {
    
    brick.header.frame_id = move_group.getEndEffectorLink(); // reference to end-effector frame
    brick.id = "brick"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_brick; // Define UGV_body dimension (in meter)
    primitive_brick.type = primitive_brick.BOX;
    primitive_brick.dimensions.resize(3);
    primitive_brick.dimensions[0] = 0.20;  // length (x)
    primitive_brick.dimensions[2] = 0.20;  // width  (y)

    switch (color)
    {
    case RED:
      primitive_brick.dimensions[1] = 0.30;  // Box length in meter
      break;
    case GREEN:
      primitive_brick.dimensions[1] = 0.60;  
      break;
    case BLUE:
      primitive_brick.dimensions[1] = 1.20;  
      break;
    default:
      primitive_brick.dimensions[1] = 1.80;  
      break;
    }
    

    geometry_msgs::Pose pose_brick; // Define a pose for the UGV_body (specified relative to frame_id)
    // q.setRPY(0, 0, 0);
    // Rotate 45 degree about x-axis from https://quaternions.online/
    pose_brick.orientation.x = 0.0;
    pose_brick.orientation.y = 0.0;
    pose_brick.orientation.z = 0.0;
    pose_brick.orientation.w = 1.0;
    pose_brick.position.x = primitive_brick.dimensions[0]/2 + DIST_EE_TO_MAGNET;
    pose_brick.position.y = 0;
    pose_brick.position.z = 0;

    brick.primitives.push_back(primitive_brick);
    brick.primitive_poses.push_back(pose_brick);
    brick.operation = brick.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_robot_body2;
    collision_robot_body2.push_back(brick);
    planning_scene_interface.addCollisionObjects(collision_robot_body2); //add the collision object into the world
    ros::Duration(0.5).sleep();
    move_group.attachObject(brick.id); // attach the magnet panel to end-effector
  }


  void detachBrick()
  {
    move_group.detachObject(brick.id);
  }


  void deleteObject()
  {
    std::vector<std::string> object_ids;
    object_ids.push_back(brick.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
  }


  void moveFromCurrentState(float toX, float toY, float toZ)
  {
    /*  Move the end_effector of UR5 to the target point seen from 
    *   Camera.
    *   INPUT: x, y, z distance w.r.t to camera axis
    */

    std::vector<geometry_msgs::Pose> waypoints_down;
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    // move UR5 to the target point seen in Camera frame
    // moving +toX => +Y in robot frame
    // moving +toY => +X in robot frame
    // moving +toZ => -Z in robot frame
    target_pose.position.x = target_pose.position.x + toY;

    // Measure DIST_CAM_TO_EE, see CONSTANT part => read bricks' position from camera, but we want to move EE to the brick
    target_pose.position.y = target_pose.position.y + toX;
    // Measure DIST_CAM_TO_EE, see CONSTANT part
    target_pose.position.z = target_pose.position.z + toZ;
    waypoints_down.push_back(target_pose);


    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.4);
    move_group.setPlanningTime(PLANNING_TIMEOUT);


    moveit_msgs::RobotTrajectory trajectory_down;
    for(int i = 0; i < 3; i++)
    {
      fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
      if (fraction == 1.0)
        break;
    }
    ROS_INFO_NAMED("tutorial", "moveFromCurrentState: CartesianPath (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;


    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go down");
      // ros::Duration(DELAY).sleep();
    #endif

    move_group.execute(cartesian_plan);
  }


  void moveToDefault(int mode)
  {
    /*  Move the arm to the default position
    *   Upfront arm with end_effector looking down
    *   Use mode 0: when the target container is RIGHT container
    *   Use mode 1: when the target container is LEFT container
    */
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if (mode == 0)
    {
      joint_group_positions[0] = 0;  // Radian
      joint_group_positions[1] = -PI/2;
      joint_group_positions[2] = -PI/180*60;
      joint_group_positions[3] = PI + PI/180 * 60;
      joint_group_positions[4] = PI/2;
      joint_group_positions[5] = 0;
    }else
    {
      joint_group_positions[0] = PI;  // Radian
      joint_group_positions[1] = -PI/2;
      joint_group_positions[2] = PI/180*60;
      joint_group_positions[3] = 2 * PI - PI/180 * 60;
      joint_group_positions[4] = -PI/2;
      joint_group_positions[5] = 0;
      
    }
    

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.25);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);


    bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
    ROS_INFO_NAMED("tutorial", "moveToDefault: %s", success ? "SUCCEEDED" : "FAILED");

    // #ifdef DEBUG
    //   visual_tools.prompt("Press 'next' to front position");
    // #endif

    move_group.move();                      // BLOCKING FUNCTION
    moveToDefault_finished_flag_msg.data = true;
    ROS_INFO("publish moveToDefault_finished_flag_msg");
    moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
  }


  bool moveXYZSlowly(float X, float Y, float Z, float max_v_scaling, float max_a_scaling)
  {
    /* This function moves the end_effector slowly in Cartesian plane
    *  The input XYZ is the distance with respect to UGV frame
    */
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_down;
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    // move according to the robot frame
    target_pose.position.x += X;
    target_pose.position.y += Y;
    target_pose.position.z += Z; 
    waypoints_down.push_back(target_pose);

    // Seong) Set planner, Max velo and Planning time
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalOrientationTolerance(0.0001);
    move_group.setGoalPositionTolerance(0.0001);


    moveit_msgs::RobotTrajectory trajectory_down;
    float eef_step_temp = 0.001;  // resolution of 1 mm
    for(int i = 0; i < 3; i++)
    {
      fraction = move_group.computeCartesianPath(waypoints_down, eef_step_temp, jump_threshold, trajectory_down);
      if (fraction == 1.0)
        break;
    }
    // if (fraction != 1.0)
    //   return false;

    // ================================= modify the velocity of Cartesian path ================================= //
    // First create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_down);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, max_v_scaling, max_a_scaling);
    rt.getRobotTrajectoryMsg(trajectory_down);

    ROS_INFO_NAMED("tutorial", "moveXYZSlowly (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to moveXYZSlowly");
    #endif

    move_group.execute(cartesian_plan);
    return true;
  }


  void moveYawSlowly(bool direction, float max_v_scaling, float max_a_scaling)
  {
    /* This function yaws the end_effector slowly in Cartesian plane
    *  + rad: left, - rad : right
    */
    if(FLAG_YAW_STOP == true)
      return;
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if (direction == LEFT)
      joint_group_positions[5] -= PI/2;
    else
      joint_group_positions[5] += PI/2;

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(max_v_scaling);
    move_group.setMaxAccelerationScalingFactor(max_a_scaling);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);

    bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
    ROS_INFO_NAMED("tutorial", "moveYawSlowly %s", success ? "SUCCEEDED" : "FAILED");

    // #ifdef DEBUG
    //   visual_tools.prompt("Press 'next' to front position");
    // #endif

    if(FLAG_YAW_STOP == true)
      return;
    move_group.move();                      // BLOCKING FUNCTION
  }


  void resetYaw()
  {
    /* This function reset yaws angle
    */
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[5] = 0;

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);

    bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
    ROS_INFO_NAMED("tutorial", "resetYaw: moving back to default yaw %s", success ? "SUCCEEDED" : "FAILED");

    // #ifdef DEBUG
    //   visual_tools.prompt("Press 'next' to front position");
    // #endif

    move_group.move();                      // BLOCKING FUNCTION
  }

  // ==================== Initialization =====================//
  void initWall()
  {
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // ============================== Set Robot Body Collision ============================== //

    // ---------- UGV_base
    moveit_msgs::CollisionObject UGV_base; // Define a collision object ROS message.
    UGV_base.header.frame_id = move_group.getPlanningFrame();

    UGV_base.id = "UGV_base"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_UGV_base; // Define UGV_base dimension (in meter)
    primitive_UGV_base.type = primitive_UGV_base.BOX;
    primitive_UGV_base.dimensions.resize(3);
    primitive_UGV_base.dimensions[0] = 0.80;  // x right
    primitive_UGV_base.dimensions[1] = 1.12;  // y front
    primitive_UGV_base.dimensions[2] = 0.59;  // z up

    geometry_msgs::Pose pose_UGV_base; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_UGV_base.orientation.x = q[0];
    pose_UGV_base.orientation.y = q[1];
    pose_UGV_base.orientation.z = q[2];
    pose_UGV_base.orientation.w = q[3];
    pose_UGV_base.position.y = -primitive_UGV_base.dimensions[1]/2 + 0.05/2 + 0.04;
    pose_UGV_base.position.z = -primitive_UGV_base.dimensions[2]/2;

    UGV_base.primitives.push_back(primitive_UGV_base);
    UGV_base.primitive_poses.push_back(pose_UGV_base);
    UGV_base.operation = UGV_base.ADD;

    // ---------- UGV_base2
    moveit_msgs::CollisionObject UGV_base2; // Define a collision object ROS message.
    UGV_base2.header.frame_id = move_group.getPlanningFrame();

    UGV_base2.id = "UGV_base2"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_UGV_base2; // Define UGV_base dimension (in meter)
    primitive_UGV_base2.type = primitive_UGV_base2.BOX;
    primitive_UGV_base2.dimensions.resize(3);
    primitive_UGV_base2.dimensions[0] = primitive_UGV_base.dimensions[0];  // x right
    primitive_UGV_base2.dimensions[1] = 0.12;  // y front
    // primitive_UGV_base2.dimensions[2] = 0.04;  // z up
    primitive_UGV_base2.dimensions[2] = 0.59;  // z up

    geometry_msgs::Pose pose_UGV_base2; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_UGV_base2.orientation.x = q[0];
    pose_UGV_base2.orientation.y = q[1];
    pose_UGV_base2.orientation.z = q[2];
    pose_UGV_base2.orientation.w = q[3];
    pose_UGV_base2.position.y = primitive_UGV_base2.dimensions[1]/2 + 0.05/2;
    // pose_UGV_base2.position.z = primitive_UGV_base2.dimensions[2]/2 - 0.29;
    pose_UGV_base2.position.z = -primitive_UGV_base2.dimensions[2]/2;

    UGV_base2.primitives.push_back(primitive_UGV_base2);
    UGV_base2.primitive_poses.push_back(pose_UGV_base2);
    UGV_base2.operation = UGV_base2.ADD;

    // ---------- Belt Motor
    moveit_msgs::CollisionObject belt_motor; // Define a collision object ROS message.
    belt_motor.header.frame_id = move_group.getPlanningFrame();

    belt_motor.id = "belt_motor"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_belt_motor; // Define UGV_base dimension (in meter)
    primitive_belt_motor.type = primitive_belt_motor.BOX;
    primitive_belt_motor.dimensions.resize(3);
    primitive_belt_motor.dimensions[0] = 0.15;  // x right
    primitive_belt_motor.dimensions[1] = 0.17;  // y front
    primitive_belt_motor.dimensions[2] = 0.10;  // z up

    geometry_msgs::Pose pose_belt_motor; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_belt_motor.orientation.x = q[0];
    pose_belt_motor.orientation.y = q[1];
    pose_belt_motor.orientation.z = q[2];
    pose_belt_motor.orientation.w = q[3];
    pose_belt_motor.position.x = primitive_belt_motor.dimensions[0]/2 - 0.02;
    pose_belt_motor.position.y = primitive_belt_motor.dimensions[1]/2;
    pose_belt_motor.position.z = primitive_belt_motor.dimensions[2]/2;

    belt_motor.primitives.push_back(primitive_belt_motor);
    belt_motor.primitive_poses.push_back(pose_belt_motor);
    belt_motor.operation = belt_motor.ADD;

    // ---------- Magnet Panel at end effector
    moveit_msgs::CollisionObject magnet_panel;
    magnet_panel.header.frame_id = move_group.getEndEffectorLink(); // reference to end-effector frame
    magnet_panel.id = "magnet_panel"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_magnet_panel; // Define UGV_body dimension (in meter)
    primitive_magnet_panel.type = primitive_magnet_panel.BOX;
    primitive_magnet_panel.dimensions.resize(3);
    primitive_magnet_panel.dimensions[0] = DIST_EE_TO_MAGNET;  // height
    primitive_magnet_panel.dimensions[1] = 0.21;  // length
    primitive_magnet_panel.dimensions[2] = 0.075;  // width

    // magnetic panel
    geometry_msgs::Pose pose_magnet_panel; // Define a pose for the UGV_body (specified relative to frame_id)
    // q.setRPY(45, 0, 0);
    // q.normalize();
    // Rotate 45 degree about x-axis from https://quaternions.online/
    pose_magnet_panel.orientation.x = 0.0;
    pose_magnet_panel.orientation.y = 0.0;
    pose_magnet_panel.orientation.z = 0.0;
    pose_magnet_panel.orientation.w = 1.0;

    //note the axis of EE
    pose_magnet_panel.position.x = primitive_magnet_panel.dimensions[0]/2;
    pose_magnet_panel.position.y = 0;
    pose_magnet_panel.position.z = 0;

    magnet_panel.primitives.push_back(primitive_magnet_panel);
    magnet_panel.primitive_poses.push_back(pose_magnet_panel);
    magnet_panel.operation = magnet_panel.ADD;

    // ---------- Container_left_out
    moveit_msgs::CollisionObject Container_left_out; // Define a collision object ROS message.
    Container_left_out.header.frame_id = move_group.getPlanningFrame();

    Container_left_out.id = "Container_left_out"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_left_out; // Define UGV_base dimension (in meter)
    primitive_Container_left_out.type = primitive_Container_left_out.BOX;
    primitive_Container_left_out.dimensions.resize(3);
    primitive_Container_left_out.dimensions[0] = 0.03;  // x right
    primitive_Container_left_out.dimensions[1] = 1.20;  // y front
    primitive_Container_left_out.dimensions[2] = 1.19;  // z up

    geometry_msgs::Pose pose_Container_left_out; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_left_out.orientation.x = q[0];
    pose_Container_left_out.orientation.y = q[1];
    pose_Container_left_out.orientation.z = q[2];
    pose_Container_left_out.orientation.w = q[3];
    pose_Container_left_out.position.x = -primitive_Container_left_out.dimensions[0]/2 - primitive_UGV_base.dimensions[0] / 2 - 0.29;
    pose_Container_left_out.position.y = -primitive_Container_left_out.dimensions[1]/2 + 0.05/2 + 0.08 + 0.04;
    pose_Container_left_out.position.z = primitive_Container_left_out.dimensions[2]/2 - 0.59;

    Container_left_out.primitives.push_back(primitive_Container_left_out);
    Container_left_out.primitive_poses.push_back(pose_Container_left_out);
    Container_left_out.operation = Container_left_out.ADD;

    // ---------- Container_left_in
    moveit_msgs::CollisionObject Container_left_in; // Define a collision object ROS message.
    Container_left_in.header.frame_id = move_group.getPlanningFrame();

    Container_left_in.id = "Container_left_in"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_left_in; // Define UGV_base dimension (in meter)
    primitive_Container_left_in.type = primitive_Container_left_in.BOX;
    primitive_Container_left_in.dimensions.resize(3);
    primitive_Container_left_in.dimensions[0] = 0.03;  // x right
    primitive_Container_left_in.dimensions[1] = 0.95;  // y front
    primitive_Container_left_in.dimensions[2] = 0.63;  // z up

    geometry_msgs::Pose pose_Container_left_in; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_left_in.orientation.x = q[0];
    pose_Container_left_in.orientation.y = q[1];
    pose_Container_left_in.orientation.z = q[2];
    pose_Container_left_in.orientation.w = q[3];
    pose_Container_left_in.position.x = -primitive_Container_left_in.dimensions[0]/2 - primitive_UGV_base.dimensions[0] / 2;
    pose_Container_left_in.position.y = -0.58 - 0.15;
    pose_Container_left_in.position.z = primitive_Container_left_in.dimensions[2]/2 - 0.59;

    Container_left_in.primitives.push_back(primitive_Container_left_in);
    Container_left_in.primitive_poses.push_back(pose_Container_left_in);
    Container_left_in.operation = Container_left_in.ADD;

    // ---------- Container_left_low_in
    moveit_msgs::CollisionObject Container_left_low_in; // Define a collision object ROS message.
    Container_left_low_in.header.frame_id = move_group.getPlanningFrame();

    Container_left_low_in.id = "Container_left_low_in"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_left_low_in; // Define UGV_base dimension (in meter)
    primitive_Container_left_low_in.type = primitive_Container_left_low_in.BOX;
    primitive_Container_left_low_in.dimensions.resize(3);
    primitive_Container_left_low_in.dimensions[0] = 0.03;  // x right
    primitive_Container_left_low_in.dimensions[1] = primitive_UGV_base.dimensions[1];  // y front
    primitive_Container_left_low_in.dimensions[2] = 0.63;  // z up

    geometry_msgs::Pose pose_Container_left_low_in; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_left_low_in.orientation.x = q[0];
    pose_Container_left_low_in.orientation.y = q[1];
    pose_Container_left_low_in.orientation.z = q[2];
    pose_Container_left_low_in.orientation.w = q[3];
    pose_Container_left_low_in.position.x = pose_Container_left_in.position.x;
    pose_Container_left_low_in.position.y = pose_UGV_base.position.y;
    pose_Container_left_low_in.position.z = primitive_Container_left_low_in.dimensions[2]/2 - 0.59;

    Container_left_low_in.primitives.push_back(primitive_Container_left_low_in);
    Container_left_low_in.primitive_poses.push_back(pose_Container_left_low_in);
    Container_left_low_in.operation = Container_left_low_in.ADD;

    // ---------- Container_right_out
    moveit_msgs::CollisionObject Container_right_out; // Define a collision object ROS message.
    Container_right_out.header.frame_id = move_group.getPlanningFrame();

    Container_right_out.id = "Container_right_out"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_right_out; // Define UGV_base dimension (in meter)
    primitive_Container_right_out.type = primitive_Container_left_out.BOX;
    primitive_Container_right_out.dimensions.resize(3);
    primitive_Container_right_out.dimensions[0] = 0.03;  // x right
    primitive_Container_right_out.dimensions[1] = 1.20;  // y front
    primitive_Container_right_out.dimensions[2] = 1.19;  // z up

    geometry_msgs::Pose pose_Container_right_out; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_right_out.orientation.x = q[0];
    pose_Container_right_out.orientation.y = q[1];
    pose_Container_right_out.orientation.z = q[2];
    pose_Container_right_out.orientation.w = q[3];
    pose_Container_right_out.position.x = primitive_Container_right_out.dimensions[0]/2 + primitive_UGV_base.dimensions[0] / 2 + 0.29;
    pose_Container_right_out.position.y = -primitive_Container_right_out.dimensions[1]/2 + 0.05/2 + 0.08 + 0.04;
    pose_Container_right_out.position.z = primitive_Container_right_out.dimensions[2]/2 - 0.59;

    Container_right_out.primitives.push_back(primitive_Container_right_out);
    Container_right_out.primitive_poses.push_back(pose_Container_right_out);
    Container_right_out.operation = Container_right_out.ADD;

    // ---------- Container_right_in
    moveit_msgs::CollisionObject Container_right_in; // Define a collision object ROS message.
    Container_right_in.header.frame_id = move_group.getPlanningFrame();

    Container_right_in.id = "Container_right_in"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_right_in; // Define UGV_base dimension (in meter)
    primitive_Container_right_in.type = primitive_Container_left_in.BOX;
    primitive_Container_right_in.dimensions.resize(3);
    primitive_Container_right_in.dimensions[0] = 0.03;  // x right
    primitive_Container_right_in.dimensions[1] = 0.95;  // y front
    primitive_Container_right_in.dimensions[2] = 0.63;  // z up

    geometry_msgs::Pose pose_Container_right_in; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_right_in.orientation.x = q[0];
    pose_Container_right_in.orientation.y = q[1];
    pose_Container_right_in.orientation.z = q[2];
    pose_Container_right_in.orientation.w = q[3];
    pose_Container_right_in.position.x = primitive_Container_right_in.dimensions[0]/2 + primitive_UGV_base.dimensions[0] / 2;
    pose_Container_right_in.position.y = -0.58 - 0.15;
    pose_Container_right_in.position.z = primitive_Container_right_in.dimensions[2]/2 - 0.59;

    Container_right_in.primitives.push_back(primitive_Container_right_in);
    Container_right_in.primitive_poses.push_back(pose_Container_right_in);
    Container_right_in.operation = Container_right_in.ADD;

    // ---------- Container_right_low_in
    moveit_msgs::CollisionObject Container_right_low_in; // Define a collision object ROS message.
    Container_right_low_in.header.frame_id = move_group.getPlanningFrame();

    Container_right_low_in.id = "Container_right_low_in"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_Container_right_low_in; // Define UGV_base dimension (in meter)
    primitive_Container_right_low_in.type = primitive_Container_right_low_in.BOX;
    primitive_Container_right_low_in.dimensions.resize(3);
    primitive_Container_right_low_in.dimensions[0] = 0.03;  // x right
    primitive_Container_right_low_in.dimensions[1] = primitive_UGV_base.dimensions[1];  // y front
    primitive_Container_right_low_in.dimensions[2] = 0.63;  // z up

    geometry_msgs::Pose pose_Container_right_low_in; // Define a pose for the Robot_bottom (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_Container_right_low_in.orientation.x = q[0];
    pose_Container_right_low_in.orientation.y = q[1];
    pose_Container_right_low_in.orientation.z = q[2];
    pose_Container_right_low_in.orientation.w = q[3];
    pose_Container_right_low_in.position.x = pose_Container_right_in.position.x;
    pose_Container_right_low_in.position.y = pose_UGV_base.position.y;
    pose_Container_right_low_in.position.z = primitive_Container_right_in.dimensions[2]/2 - 0.59;

    Container_right_low_in.primitives.push_back(primitive_Container_right_low_in);
    Container_right_low_in.primitive_poses.push_back(pose_Container_right_low_in);
    Container_right_low_in.operation = Container_right_low_in.ADD;

    // ---------- ground
    moveit_msgs::CollisionObject ground; // Define a collision object ROS message.
    ground.header.frame_id = move_group.getPlanningFrame(); // reference to end-effector frame
    ground.id = "ground"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_ground; // Define UGV_body dimension (in meter)
    primitive_ground.type = primitive_ground.BOX;
    primitive_ground.dimensions.resize(3);
    primitive_ground.dimensions[0] = 3;  // length (x)
    primitive_ground.dimensions[1] = 3;  // width  (y)
    primitive_ground.dimensions[2] = 0.001;  // height (z)

    geometry_msgs::Pose pose_ground; // Define a pose for the UGV_body (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_ground.orientation.x = q[0];
    pose_ground.orientation.y = q[1];
    pose_ground.orientation.z = q[2];
    pose_ground.orientation.w = q[3];
    pose_ground.position.x = 0;
    pose_ground.position.y = 0;
    pose_ground.position.z = - primitive_ground.dimensions[2]/2 - (primitive_UGV_base.dimensions[2]);

    ground.primitives.push_back(primitive_ground);
    ground.primitive_poses.push_back(pose_ground);
    ground.operation = ground.ADD;

    // ---------- ground_container
    moveit_msgs::CollisionObject ground_container; // Define a collision object ROS message.
    ground_container.header.frame_id = move_group.getPlanningFrame(); // reference to end-effector frame
    ground_container.id = "ground_container"; // The id of the object is used to identify it.

    shape_msgs::SolidPrimitive primitive_ground_container; // Define UGV_body dimension (in meter)
    primitive_ground_container.type = primitive_ground_container.BOX;
    primitive_ground_container.dimensions.resize(3);
    primitive_ground_container.dimensions[0] = 3;  // length (x)
    primitive_ground_container.dimensions[1] = primitive_UGV_base.dimensions[1];  // width  (y)
    primitive_ground_container.dimensions[2] = 0.001;  // height (z)

    geometry_msgs::Pose pose_ground_container; // Define a pose for the UGV_body (specified relative to frame_id)
    q.setRPY(0, 0, 0);
    pose_ground_container.orientation.x = q[0];
    pose_ground_container.orientation.y = q[1];
    pose_ground_container.orientation.z = q[2];
    pose_ground_container.orientation.w = q[3];
    pose_ground_container.position.x = 0;
    pose_ground_container.position.y = -primitive_UGV_base.dimensions[1]/2 + 0.05/2 + 0.04;
    pose_ground_container.position.z = primitive_ground_container.dimensions[2]/2 - 0.43;

    ground_container.primitives.push_back(primitive_ground_container);
    ground_container.primitive_poses.push_back(pose_ground_container);
    ground_container.operation = ground_container.ADD;

    // ---------- Collect Whole Robot Body

    std::vector<moveit_msgs::CollisionObject> collision_robot_body;
    collision_robot_body.push_back(UGV_base);
    collision_robot_body.push_back(UGV_base2);
    collision_robot_body.push_back(belt_motor);
    collision_robot_body.push_back(magnet_panel);
    collision_robot_body.push_back(Container_left_out);
    collision_robot_body.push_back(Container_left_in);
    collision_robot_body.push_back(Container_left_low_in);
    collision_robot_body.push_back(Container_right_out);
    collision_robot_body.push_back(Container_right_in);
    collision_robot_body.push_back(Container_right_low_in);
    collision_robot_body.push_back(ground);
    collision_robot_body.push_back(ground_container);
    planning_scene_interface.addCollisionObjects(collision_robot_body); //add the collision object into the world
    ros::Duration(DELAY).sleep(); // wait to build the object before attaching to ee
    move_group.attachObject(magnet_panel.id); // attach the magnet panel to end-effector


  }

  // ==================== FOR DEBUGGING REMOVE IF NOT NEEDED ==================== //
  void manual_moveXYZ(const geometry_msgs::Pose::ConstPtr& msg)
  {
    target_pose = move_group.getCurrentPose().pose;
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    std::vector<geometry_msgs::Pose> waypoints_down;

    // move according to the robot frame
    // +X: right
    // +Y: front
    // +Z: up
    target_pose.position.x =  target_pose.position.x + msg->position.x;
    target_pose.position.y =  target_pose.position.y + msg->position.y; // Measure DIST_CAM_TO_EE, see CONSTANT part

    // Measure DIST_CAM_TO_EE, see CONSTANT part => read button's position from camera, but we want to move EE to the button
    target_pose.position.z =  target_pose.position.z + msg->position.z;
    waypoints_down.push_back(target_pose);

    // Seong) Set planner, Max velo and Planning time
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);


    moveit_msgs::RobotTrajectory trajectory_down;
    for(int i = 0; i < 3; i++)
    {
      fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
      if (fraction == 1.0)
        break;
    }
    // ================================= modify the velocity of Cartesian path ================================= //
    // First create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_down);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, 0.3, 0.3);
    rt.getRobotTrajectoryMsg(trajectory_down);
    ROS_INFO_NAMED("tutorial", "manual_moveXYZ CartesianPath (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go down");
    #endif

    move_group.execute(cartesian_plan);
    // ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed
  }


  void moveToStorageSideFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    // Subscribe: moveToStorageSide_flag_msg
    // Publish: moveToStorageSide_finished_flag_msg
    if (msg->data == true){
      gb_count_box += 1;
      moveToStorageSide_DEBUG(gb_count_box);

      if (gb_count_box == 5)
      { // Can only store 5 layers in the container
        gb_count_box = 0;
      }
    }else{
      // DO NOTHING
    }
  }


  void moveToStorageSide_DEBUG(uint box_count)
  {
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //  ====================== Turn the robot arm to the correct container side  ====================== //
    {
      current_state = move_group.getCurrentState();
      ros::Duration(0.5).sleep();
      current_state = move_group.getCurrentState();
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 3 * PI/2; // Turn UR toward the container LEFT side

      move_group.setJointValueTarget(joint_group_positions);
      move_group.setPlanningTime(PLANNING_TIMEOUT);
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxVelocityScalingFactor(0.3);
      move_group.setMaxAccelerationScalingFactor(0.3);

      bool success{move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS};
      ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 2: Turn %s", success ? "SUCCEEDED" : "FAILED");


      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to front position");
      #endif

      move_group.move();                      // BLOCKING FUNCTION
    }

    //  ====================== go to the container  ====================== //
    {
      std::vector<geometry_msgs::Pose> waypoints_to_storage;
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
      ros::Duration(0.5).sleep();
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

      target_pose.position.x = -0.565;
      target_pose.position.y = -0.1;
      target_pose.position.z = 0.65;

      waypoints_to_storage.push_back(target_pose);
      
      target_pose.position.z = (box_count - 1) * 0.22;
      ROS_INFO("z = %lf", target_pose.position.z);

      waypoints_to_storage.push_back(target_pose);

      move_group.setMaxVelocityScalingFactor(0.2);
      move_group.setMaxAccelerationScalingFactor(0.2);
      move_group.setPlanningTime(PLANNING_TIMEOUT);

      moveit_msgs::RobotTrajectory trajectory_to_storage;
      for(int i = 0; i < 3; i++)
      {
        fraction = move_group.computeCartesianPath(waypoints_to_storage, eef_step, jump_threshold, trajectory_to_storage);
        if (fraction == 1.0)
          break;
      }
      // ================================= modify the velocity of Cartesian path ================================= //
      // First create a RobotTrajectory object
      robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_to_storage);
      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      // Fourth compute computeTimeStamps
      bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);
      rt.getRobotTrajectoryMsg(trajectory_to_storage);

      ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 4: Cartesian To Storage (%.2f%% achieved)", fraction * 100.0);
      cartesian_plan.trajectory_ = trajectory_to_storage;

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishPath(waypoints_to_storage, rvt::LIME_GREEN, rvt::SMALL);
      for (std::size_t i = 0; i < waypoints_to_storage.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints_to_storage[i], "pt" + std::to_string(i), rvt::SMALL);
      visual_tools.trigger();

      #ifdef DEBUG
        visual_tools.prompt("Press 'next' to go down");
      #endif

      move_group.execute(cartesian_plan);
    }

    //  ====================== turn off the magnet to detach the brick  ====================== //
    {
      magnet_state_msg.data = false;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET OFF
      ROS_INFO("_placeInContainerServiceCallback: MAGNET_OFF");
      ros::Duration(5).sleep();
    }

    //  ====================== move back to the default position  ====================== //
    {
      std::vector<geometry_msgs::Pose> waypoints_up_from_storage;
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
      ros::Duration(0.5).sleep();
      target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
      target_pose.position.z = 0.30;
      waypoints_up_from_storage.push_back(target_pose);
      move_group.execute(cartesian_plan);
      move_group.setPlanningTime(PLANNING_TIMEOUT);
      moveit_msgs::RobotTrajectory trajectory_up_from_storage;
      for(int i = 0; i < 3; i++)
      {
        fraction = move_group.computeCartesianPath(waypoints_up_from_storage, eef_step, jump_threshold, trajectory_up_from_storage);
        if (fraction == 1.0)
          break;
      }
      ROS_INFO_NAMED("tutorial", "trajectory_up_from_storage: Cartesian To Storage (%.2f%% achieved)", fraction * 100.0);
      cartesian_plan.trajectory_ = trajectory_up_from_storage;

      move_group.execute(cartesian_plan);

      moveToDefault(RIGHT);
      
      moveToDefault_finished_flag_msg.data = true;
      moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
    } 

  }


};  // end of class definition

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbzirc_ur5_control");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  Arm mbzircArm;
  ros::waitForShutdown();

  return 0;
}
