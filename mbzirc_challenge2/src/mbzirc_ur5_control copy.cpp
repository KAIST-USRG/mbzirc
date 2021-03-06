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
#include "mission_manager/ur_move.h"
#include "mbzirc_challenge2/go_to_brick.h"

// msg
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>


// #define DEBUG
#define DELAY 1.0         // for sleep function => robot updating states => 0.4 s fail (?)
#define PLANNING_TIMEOUT 2
#define NUM_SUM 3      // to average the pose message
#define NUM_DISCARD 10
#define DIST_EE_TO_MAGNET 0.0750
#define DIST_CAM_TO_EE 0
#define DIST_LIDAR_TO_MAGNET 0
#define Z_OFFSET 0.02
#define BELT_BASE_HEIGHT 0.26

#define RED 4
#define GREEN 3
#define BLUE 2
#define ORANGE 1


namespace rvt = rviz_visual_tools;

// for rotation of objects
tf2::Quaternion q;

// FLAG for robot motion

bool FLAG_READ_CAM_DATA = false;    // Should the arm read message from camera?
bool FLAG_SWITCH_TOUCHED = false;
bool FLAG_SWITCH_RESET = true;
bool FLAG_MAGNET_ON = true;      // Is the magnet on?


int gb_count_pose_msg = 0;
int gb_discard_noise = 0;
int gb_count_move = 0;  // 0 for moveXY, 1 for moveZ
int gb_count_box = 0;
float gb_x_sum = 0;
float gb_y_sum = 0;
float gb_z_sum = 0;
float gb_xq_sum = 0;
float gb_yq_sum = 0;
float gb_zq_sum = 0;
float gb_wq_sum = 0;



class Arm{
private:
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  moveit_msgs::CollisionObject brick; // Define a collision object ROS message.


  // for checking success of planning
  bool success{};

  // for Cartesian Path
  const double jump_threshold = 0.0; // read only.
  const double eef_step = 0.01;      // read only.
  double PI = 3.141592653589793;
  double fraction;

  // Publisher
  ros::Publisher moveToStorageSide_finished_flag_pub;
  ros::Publisher moveToDefault_finished_flag_pub;
  ros::Publisher avg_pose_pub;
  ros::Publisher box_count_pub;
  ros::Publisher magnet_state_pub;
  ros::Publisher readCamData_flag_pub;

  // Subscriber
  ros::Subscriber pose_from_cam_sub;
  ros::Subscriber moveToStorageSide_flag_sub;
  ros::Subscriber moveToDefault_flag_sub;
  ros::Subscriber moveXYZ_sub;
  ros::Subscriber readCamData_flag_sub;
  ros::Subscriber manual_moveXVZ_sub;
  ros::Subscriber magnet_state_sub;
  ros::Subscriber sensor_range_sub;
  ros::Subscriber switch_state_sub;

  // msg
  std_msgs::Bool moveToStorageSide_flag_msg;
  std_msgs::Bool readCamData_flag_msg;
  std_msgs::Bool moveToDefault_flag_msg;
  std_msgs::UInt16 box_count_msg;
  std_msgs::Bool magnet_state_msg;
  std_msgs::Bool moveToStorageSide_finished_flag_msg;
  std_msgs::Bool moveToDefault_finished_flag_msg;

  // ServiceClient
  ros::ServiceClient go_to_brick_sc;

  // ServiceServer
  ros::ServiceServer ur_move_ss;
  ros::ServiceServer go_to_brick_ss;

  // Service
  mbzirc_challenge2::go_to_brick go_to_brick_srv;

  ros::NodeHandle node_handle;
  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroupInterface move_group;
  moveit_visual_tools::MoveItVisualTools visual_tools;
  moveit::core::RobotStatePtr current_state;
  geometry_msgs::Pose target_pose;


  const robot_state::JointModelGroup* joint_model_group;
  Eigen::Isometry3d text_pose;

  geometry_msgs::Pose avg_pose_msg; // absolute position w.r.t UR5 base



public:

  Arm():nh("~"), move_group("manipulator"), visual_tools("base_link") {

    // Initialization
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

    // Node Communications

    moveToStorageSide_finished_flag_pub = nh.advertise<std_msgs::Bool>("/moveToStorageSide_finish_flag", 10);
    moveToDefault_finished_flag_pub = nh.advertise<std_msgs::Bool>("/moveToDefault_finish_flag", 10);
    avg_pose_pub = nh.advertise<geometry_msgs::Pose>("/avg_pose", 10);
    box_count_pub = nh.advertise<std_msgs::UInt16>("/box_count", 10);
    magnet_state_pub = nh.advertise<std_msgs::Bool>("/magnet_on", 10);
    readCamData_flag_pub = nh.advertise<std_msgs::Bool>("/readCamData_Flag", 10);

    pose_from_cam_sub = nh.subscribe("/brick_pose", 10, &Arm::calcAvgCallback, this);
    moveToStorageSide_flag_sub = nh.subscribe("/moveToStorageSide_flag", 10, &Arm::moveToStorageSideFlagCallback, this);
    moveToDefault_flag_sub = nh.subscribe("/moveToDefault_flag", 10, &Arm::moveToDafaultFlagCallback, this);
    // moveXYZ_sub = nh.subscribe("/avg_pose", 10, &Arm::_moveXYZCallback, this);
    readCamData_flag_sub = nh.subscribe("/readCamData_Flag", 100, &Arm::readCamDataFlagCallback, this);
    manual_moveXVZ_sub = nh.subscribe("/manual_moveXVZ", 10, &Arm::manual_moveXYZ, this);
    // sensor_range_sub = nh.subscribe("/sensor_range", 1, &Arm::moveDownCallBack, this);
    switch_state_sub = nh.subscribe("/switch_state", 1, &Arm::switchStateCallback, this);

    // Service Client
    go_to_brick_sc = nh.serviceClient<mbzirc_challenge2::go_to_brick>("go_to_brick");
    // Service Server
    ur_move_ss = nh.advertiseService("ur_move", &Arm::test, this);
    go_to_brick_ss = nh.advertiseService("go_to_brick", &Arm::_moveXYZCallback, this);

    ros::Duration(1).sleep();
    // Turn on Magnet
    magnet_state_msg.data = true;
    magnet_state_pub.publish(magnet_state_msg);
    ROS_INFO("MAGNET_ON");

  }

  bool test(mission_manager::ur_move::Request  &req,
            mission_manager::ur_move::Response  &res)
  {    
    // read_data_from_camera
    readCamData_flag_msg.data = true;
    readCamData_flag_pub.publish(readCamData_flag_msg);

    geometry_msgs::PoseConstPtr brick_pose_msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/avg_pose");
    // std_msgs::BoolConstPtr test_msg = ros::topic::waitForMessage<std_msgs::Bool>("/moveToDefault_finish_flag");

    ROS_INFO("brick_pose_msg: x = %lf, y = %lf, z = %lf", brick_pose_msg->position.x, brick_pose_msg->position.y, brick_pose_msg->position.z);
    go_to_brick_srv.request.x = brick_pose_msg->position.x;
    go_to_brick_srv.request.y = brick_pose_msg->position.y;
    go_to_brick_srv.request.z = brick_pose_msg->position.z;
    go_to_brick_srv.request.qx = brick_pose_msg->orientation.x;
    go_to_brick_srv.request.qy = brick_pose_msg->orientation.y;
    go_to_brick_srv.request.qz = brick_pose_msg->orientation.z;
    go_to_brick_srv.request.qw = brick_pose_msg->orientation.w;
    
    // Check if that position is in workspace
    // Go to get the brick
    // Go back to default
    if(go_to_brick_sc.call(go_to_brick_srv)) 
    {
      std::cout << "Response : " << go_to_brick_srv.response.success << std::endl;
      if (!go_to_brick_srv.response.success)  // failed to pick up
      {
        // do something
      }else if (!go_to_brick_srv.response.workspace_reachable) // failed to reach the brick
      {
        // Tell mission maneger
        res.workspace_reachable = false; 
        return true;
      }
    }
    else {  // fail to request service
      std::cout << "Failed to call service" << std::endl;
      return false;
    }

    // Go to container and place the brick
    

    // Go back to default
    return true;
  }

  void magnetStateCallBack(const std_msgs::Bool::ConstPtr& msg)
  {
    // keep track of the magnet state => allows arm to read data only when the magnet is on
    if (msg->data == true){
      FLAG_MAGNET_ON = true;
    }
    else if (msg->data == false){
      FLAG_MAGNET_ON = false;
    }
  }

  void switchStateCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    // keep track of the magnet state => allows arm to read data only when the magnet is on
    if (msg->data == true && FLAG_SWITCH_TOUCHED == false)
    { // The active motion plan should stop when the switch is triggered
      FLAG_SWITCH_TOUCHED = true;
      move_group.stop();
    }
    else if (msg->data == false)
    {
      FLAG_SWITCH_TOUCHED = false;  // switch is deactivated

    }else
    {
      // do nothing
    }
    
  }

  void readCamDataFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data == true){
      FLAG_READ_CAM_DATA = true;
    }else{
      FLAG_READ_CAM_DATA = false;
    }
  }


  void calcAvgCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
    // read the brick pose from the camera
    // then go and get it
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

        ROS_INFO("Accumulate the data");
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
        ROS_INFO("x = %lf, y = %lf, z = %lf", avg_pose_msg.position.x, avg_pose_msg.position.y, avg_pose_msg.position.z);

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

  // void _moveXYZCallback(const geometry_msgs::Pose::ConstPtr& msg)
  bool _moveXYZCallback(mbzirc_challenge2::go_to_brick::Request  &req,
                        mbzirc_challenge2::go_to_brick::Response  &res)
  {
    // 4 steps
    // Step 1: Rotate the magnetic panel
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // convert quaternion to roll, pitch, yaw

    tf::Quaternion q_temp{
        // Note that norm of this Quaternion needs to be ONE
        // Otherwise, it's inaccurate.
        req.qx,
        req.qy,
        req.qz,
        req.qw
        // msg->orientation.x,
        // msg->orientation.y,
        // msg->orientation.z,
        // msg->orientation.w
    };

    tf::Matrix3x3 m(q_temp);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (yaw < 0)
      yaw += PI;
    if (yaw > PI/2)
      yaw = -(PI-yaw);

    ROS_INFO("Roll = %f, Pitch = %f, Yaw = %f", roll*180./PI, pitch*180./PI, yaw*180./PI);

    joint_group_positions[5] += yaw;  // radians
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");

    if(!success)  // Don't proceed if cannot reach the position
    {
      res.workspace_reachable = false;
      res.success = false;
      return true;
    }
      
    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();


    ROS_INFO("yaw = %f", yaw*180./PI);
    #ifdef DEBUG
        visual_tools.prompt("Press 'next' to front position"); // DEBUG remove if not NEEDED
    #endif
    move_group.move(); //move to storage on left side

    // Step 2: Move close to the brick
    // input: x, y, z distance w.r.t to camera axis
    // moving +toX => +X in robot frame
    // moving +toY => -Y in robot frame
    // moving +toZ => -Z in robot frame

    std::vector<geometry_msgs::Pose> waypoints_down;
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    // target_pose.position.z = target_pose.position.z - (msg->position.z / 2);
    target_pose.position.z = target_pose.position.z - (req.z / 2);
    waypoints_down.push_back(target_pose);
    // target_pose.position.x = target_pose.position.x + msg->position.x;
    // target_pose.position.y = target_pose.position.y - msg->position.y + DIST_CAM_TO_EE;
    // target_pose.position.z = target_pose.position.z - (msg->position.z / 2) + DIST_LIDAR_TO_MAGNET + Z_OFFSET;
    target_pose.position.x = target_pose.position.x + req.x;
    target_pose.position.y = target_pose.position.y - req.y + DIST_CAM_TO_EE;
    target_pose.position.z = target_pose.position.z - (req.z / 2) + DIST_LIDAR_TO_MAGNET + Z_OFFSET;
    
    waypoints_down.push_back(target_pose);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(PLANNING_TIMEOUT);

    moveit_msgs::RobotTrajectory trajectory_down;
    fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
    ROS_INFO_NAMED("tutorial", "Step 2: Move close to the brick (%.2f%% achieved)", fraction * 100.0);
    if(fraction < 1.0)  // Don't proceed if cannot reach the position
    {
      res.workspace_reachable = false;
      res.success = false;
      return true;
    }

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

    // Step 3: moving down slowly untile the switch is triggered
    while (true)
    {
      moveDownSlowly(); // continue to move down until the tactile sensor is triggered

      if (FLAG_SWITCH_TOUCHED == true) // tactile sensor is triggered => stop
      {
        break;
      }
    }

    attachBrick(0);

    // Step 4: Move back to default position, preparing to store the brick on the UGV
    FLAG_READ_CAM_DATA = false; // Disable reading box pose data stream
    gb_count_move = 0;
    moveToDefault();
    moveToDefault_finished_flag_msg.data = true;
    moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is at default position 
    res.success = true;
    res.workspace_reachable = true;
    return true;
  }

  void moveToStorageSideFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    // Subscribe: moveToStorageSide_flag_msg
    // Publish: moveToStorageSide_finished_flag_msg
    if (msg->data == true){
      gb_count_box += 1;
      moveToStorageSide2(gb_count_box);

      magnet_state_msg.data = false;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET OFF
      ROS_INFO("MAGNET_OFF");
      ros::Duration(0.5).sleep();
      detachBrick();

      moveToStorageSide_finished_flag_msg.data = true;
      moveToStorageSide_finished_flag_pub.publish(moveToStorageSide_finished_flag_msg);

      box_count_msg.data = gb_count_box;
      box_count_pub.publish(box_count_msg); // Publish the number of boxes in the container after storing

      moveToDefault();
      magnet_state_msg.data = true;
      magnet_state_pub.publish(magnet_state_msg); // MAGNET OFF
      ROS_INFO("MAGNET_ON");
      moveToDefault_finished_flag_msg.data = true;
      moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up
      deleteObject();

      if (gb_count_box == 5)
      { // Can only store 5 layers in the container
        gb_count_box = 0;
      }
    }else{
      // DO NOTHING
    }
  }

  void moveToDafaultFlagCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    // Subscribe: moveToDefault_flag
    // Publish: moveToDefault_finish_flag
    if (msg->data == true){
      moveToDefault();
      moveToDefault_finished_flag_msg.data = true;
      moveToDefault_finished_flag_pub.publish(moveToDefault_finished_flag_msg); // let the planner know that the arm is up

    }else{
      // DO NOTHING
    }
  }

  void moveToDefault()
  {
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = PI/2;  // Radian
    joint_group_positions[1] = -PI/2;
    joint_group_positions[2] = PI/4;
    joint_group_positions[3] = 2 * PI - PI/4;
    joint_group_positions[4] = -PI/2;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);


    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "SUCCEEDED" : "FAILED");

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to front position");
    #endif

    move_group.move();                      // BLOCKING FUNCTION


  }

  void moveToStorageSide(uint box_count)
  {
    // Get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // // STEP1: Rotate Up
    // current_state = move_group.getCurrentState();
    // ros::Duration(0.5).sleep();
    // current_state = move_group.getCurrentState();

    // joint_group_positions[2] = PI/6;
    // joint_group_positions[3] = PI;

    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.setPlanningTime(PLANNING_TIMEOUT);
    // move_group.setMaxVelocityScalingFactor(0.3);
    // move_group.setMaxAccelerationScalingFactor(0.3);
    // move_group.setGoalJointTolerance(0.01);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 1: rotate up %s", success ? "SUCCEEDED" : "FAILED");
    
    // #ifdef DEBUG
    //   visual_tools.prompt("Press 'next' to front position");
    // #endif

    // move_group.move();                      // BLOCKING FUNCTION

    // STEP2: Turn Left (UGV view)
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = PI/2 + PI/2;  // Radian rotate 90 from Default Position

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);

    ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 2: Turn left %s", success ? "SUCCEEDED" : "FAILED");


    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to front position");
    #endif

    move_group.move();                      // BLOCKING FUNCTION

    // // STEP3: Rotate Down
    // current_state = move_group.getCurrentState();
    // ros::Duration(0.5).sleep();
    // current_state = move_group.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // joint_group_positions[3] = 2*PI - joint_group_positions[2];  // Make the brick face downward
    // joint_group_positions[4] = -PI/2;
    // joint_group_positions[5] = PI/4;

    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.setMaxVelocityScalingFactor(0.3);
    // move_group.setMaxAccelerationScalingFactor(0.3);
    // move_group.setPlanningTime(PLANNING_TIMEOUT);
    // move_group.setGoalJointTolerance(0.001);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 3: Rotate Down %s", success ? "SUCCEEDED" : "FAILED");

    // #ifdef DEBUG
    //   visual_tools.prompt("Press 'next' to front position");
    // #endif

    // move_group.move();                      // BLOCKING FUNCTION

    // STEP4: Go to the storage

    std::vector<geometry_msgs::Pose> waypoints_to_storage;

    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    target_pose.position.x = -0.56;
    target_pose.position.y = -0.1;
    target_pose.position.z = 0.65;

    waypoints_to_storage.push_back(target_pose);

    target_pose.position.z = (box_count - 1) * 0.22;

    waypoints_to_storage.push_back(target_pose);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(PLANNING_TIMEOUT);

    moveit_msgs::RobotTrajectory trajectory_to_storage;
    fraction = move_group.computeCartesianPath(waypoints_to_storage, eef_step, jump_threshold, trajectory_to_storage);
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

  void moveToStorageSide2(uint box_count)
  {
    // Get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // STEP2: Turn Left (UGV view)
    current_state = move_group.getCurrentState();
    ros::Duration(0.5).sleep();
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = PI/2 - PI/2;  // Radian rotate 90 from Default Position

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalJointTolerance(0.01);

    ROS_INFO_NAMED("tutorial", "moveToStorageSide Step 2: Turn left %s", success ? "SUCCEEDED" : "FAILED");


    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to front position");
    #endif

    move_group.move();                      // BLOCKING FUNCTION

    // STEP4: Go to the storage

    std::vector<geometry_msgs::Pose> waypoints_to_storage;

    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    target_pose.position.x = 0.56;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.65;

    waypoints_to_storage.push_back(target_pose);

    target_pose.position.z = (box_count - 1) * 0.22;

    waypoints_to_storage.push_back(target_pose);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(PLANNING_TIMEOUT);

    moveit_msgs::RobotTrajectory trajectory_to_storage;
    fraction = move_group.computeCartesianPath(waypoints_to_storage, eef_step, jump_threshold, trajectory_to_storage);
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

  

  void moveFromCurrentState(float toX, float toY, float toZ)
  {
    // input: x, y, z distance w.r.t to camera axis
    // moving +toX => +X in robot frame
    // moving +toY => -Y in robot frame
    // moving +toZ => -Z in robot frame
    std::vector<geometry_msgs::Pose> waypoints_down;
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    // move according to the robot frame
    // +X: right
    // +Y: front
    // +Z: up
    target_pose.position.x = target_pose.position.x + toX;

    // Measure DIST_CAM_TO_EE, see CONSTANT part => read bricks' position from camera, but we want to move EE to the brick
    target_pose.position.y = target_pose.position.y - toY;
    // Measure DIST_CAM_TO_EE, see CONSTANT part
    target_pose.position.z = target_pose.position.z - toZ;
    waypoints_down.push_back(target_pose);


    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(PLANNING_TIMEOUT);


    moveit_msgs::RobotTrajectory trajectory_down;
    fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
    ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath down (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;


    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go down");
      // ros::Duration(DELAY).sleep();
    #endif

    move_group.execute(cartesian_plan);


  }

  void moveDownSlowly()
  {
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_down;
    ros::Duration(0.5).sleep();
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position

    // move according to the robot frame
    // +X: right
    // +Y: front
    // +Z: up
    target_pose.position.z -= 0.1;  // go down by 0.005 millimeter
    waypoints_down.push_back(target_pose);

    // Seong) Set planner, Max velo and Planning time
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalOrientationTolerance(0.0001);
    move_group.setGoalPositionTolerance(0.0001);


    moveit_msgs::RobotTrajectory trajectory_down;
    float eef_step_temp = 0.001;  // resolution of 1 mm
    fraction = move_group.computeCartesianPath(waypoints_down, eef_step_temp, jump_threshold, trajectory_down);

    // First create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_down);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, 0.01, 0.01);
    rt.getRobotTrajectoryMsg(trajectory_down);

    ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath down (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go down");
    #endif

    move_group.execute(cartesian_plan);
  }


  // FOR DEBUGGING REMOVE IF NOT NEEDED
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
    target_pose.position.y =  target_pose.position.y - msg->position.y; // Measure DIST_CAM_TO_EE, see CONSTANT part

    // Measure DIST_CAM_TO_EE, see CONSTANT part => read button's position from camera, but we want to move EE to the button
    target_pose.position.z =  target_pose.position.z - msg->position.z;
    waypoints_down.push_back(target_pose);

    // Seong) Set planner, Max velo and Planning time
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);


    moveit_msgs::RobotTrajectory trajectory_down;
    fraction = move_group.computeCartesianPath(waypoints_down, 0.001, jump_threshold, trajectory_down);
    ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath down (%.2f%% achieved)", fraction * 100.0);
    cartesian_plan.trajectory_ = trajectory_down;

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go down");
    #endif

    move_group.execute(cartesian_plan);
    // ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed
  }


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
      primitive_UGV_base2.dimensions[2] = 0.04;  // z up

      geometry_msgs::Pose pose_UGV_base2; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);
      pose_UGV_base2.orientation.x = q[0];
      pose_UGV_base2.orientation.y = q[1];
      pose_UGV_base2.orientation.z = q[2];
      pose_UGV_base2.orientation.w = q[3];
      pose_UGV_base2.position.y = primitive_UGV_base2.dimensions[1]/2 + 0.05/2;
      pose_UGV_base2.position.z = primitive_UGV_base2.dimensions[2]/2 - 0.29;

      UGV_base2.primitives.push_back(primitive_UGV_base2);
      UGV_base2.primitive_poses.push_back(pose_UGV_base2);
      UGV_base2.operation = UGV_base2.ADD;

      // ---------- Magnet Panel at end effector
      moveit_msgs::CollisionObject magnet_panel;
      magnet_panel.header.frame_id = move_group.getEndEffectorLink(); // reference to end-effector frame
      magnet_panel.id = "magnet_panel"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_magnet_panel; // Define UGV_body dimension (in meter)
      primitive_magnet_panel.type = primitive_magnet_panel.BOX;
      primitive_magnet_panel.dimensions.resize(3);
      primitive_magnet_panel.dimensions[0] = DIST_EE_TO_MAGNET;  // length (x)
      primitive_magnet_panel.dimensions[1] = 0.115;  // width  (y)
      primitive_magnet_panel.dimensions[2] = 0.075;  // height (z)

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
      pose_magnet_panel.position.y = 0.02475;
      pose_magnet_panel.position.z = 0;

      magnet_panel.primitives.push_back(primitive_magnet_panel);
      magnet_panel.primitive_poses.push_back(pose_magnet_panel);
      magnet_panel.operation = magnet_panel.ADD;

      // ---------- Container1
      moveit_msgs::CollisionObject Container1; // Define a collision object ROS message.
      Container1.header.frame_id = move_group.getPlanningFrame();

      Container1.id = "Container1"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_Container1; // Define UGV_base dimension (in meter)
      primitive_Container1.type = primitive_Container1.BOX;
      primitive_Container1.dimensions.resize(3);
      primitive_Container1.dimensions[0] = 0.03;  // x right
      primitive_Container1.dimensions[1] = 1.20;  // y front
      primitive_Container1.dimensions[2] = 1.0;  // z up

      geometry_msgs::Pose pose_Container1; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);
      pose_Container1.orientation.x = q[0];
      pose_Container1.orientation.y = q[1];
      pose_Container1.orientation.z = q[2];
      pose_Container1.orientation.w = q[3];
      pose_Container1.position.x = -primitive_Container1.dimensions[0]/2 - primitive_UGV_base.dimensions[0] / 2 - 0.33;
      pose_Container1.position.y = -primitive_Container1.dimensions[1]/2 + 0.05/2 + 0.08 + 0.04;
      pose_Container1.position.z = primitive_Container1.dimensions[2]/2 - 0.30 - 0.30;

      Container1.primitives.push_back(primitive_Container1);
      Container1.primitive_poses.push_back(pose_Container1);
      Container1.operation = Container1.ADD;

      // ---------- Container2
      moveit_msgs::CollisionObject Container2; // Define a collision object ROS message.
      Container2.header.frame_id = move_group.getPlanningFrame();

      Container2.id = "Container2"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_Container2; // Define UGV_base dimension (in meter)
      primitive_Container2.type = primitive_Container2.BOX;
      primitive_Container2.dimensions.resize(3);
      primitive_Container2.dimensions[0] = 0.03;  // x right
      primitive_Container2.dimensions[1] = 0.95;  // y front
      primitive_Container2.dimensions[2] = .80;  // z up

      geometry_msgs::Pose pose_Container2; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);
      pose_Container2.orientation.x = q[0];
      pose_Container2.orientation.y = q[1];
      pose_Container2.orientation.z = q[2];
      pose_Container2.orientation.w = q[3];
      pose_Container2.position.x = -primitive_Container2.dimensions[0]/2 - 0.4;
      pose_Container2.position.y = -0.58 - 0.15;
      pose_Container2.position.z = primitive_Container2.dimensions[2]/2 - 0.30 - 0.30;

      Container2.primitives.push_back(primitive_Container2);
      Container2.primitive_poses.push_back(pose_Container2);
      Container2.operation = Container2.ADD;

      // ---------- Container3
      moveit_msgs::CollisionObject Container3; // Define a collision object ROS message.
      Container3.header.frame_id = move_group.getPlanningFrame();

      Container3.id = "Container3"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_Container3; // Define UGV_base dimension (in meter)
      primitive_Container3.type = primitive_Container1.BOX;
      primitive_Container3.dimensions.resize(3);
      primitive_Container3.dimensions[0] = 0.03;  // x right
      primitive_Container3.dimensions[1] = 1.20;  // y front
      primitive_Container3.dimensions[2] = 1.0;  // z up

      geometry_msgs::Pose pose_Container3; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);
      pose_Container3.orientation.x = q[0];
      pose_Container3.orientation.y = q[1];
      pose_Container3.orientation.z = q[2];
      pose_Container3.orientation.w = q[3];
      pose_Container3.position.x = primitive_Container3.dimensions[0]/2 + primitive_UGV_base.dimensions[0] / 2 + 0.33;
      pose_Container3.position.y = -primitive_Container3.dimensions[1]/2 + 0.05/2 + 0.08 + 0.04;
      pose_Container3.position.z = primitive_Container3.dimensions[2]/2 - 0.30 - 0.30;

      Container3.primitives.push_back(primitive_Container3);
      Container3.primitive_poses.push_back(pose_Container3);
      Container3.operation = Container3.ADD;

      // ---------- Container4
      moveit_msgs::CollisionObject Container4; // Define a collision object ROS message.
      Container4.header.frame_id = move_group.getPlanningFrame();

      Container4.id = "Container4"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_Container4; // Define UGV_base dimension (in meter)
      primitive_Container4.type = primitive_Container2.BOX;
      primitive_Container4.dimensions.resize(3);
      primitive_Container4.dimensions[0] = 0.03;  // x right
      primitive_Container4.dimensions[1] = 0.95;  // y front
      primitive_Container4.dimensions[2] = 1.00;  // z up

      geometry_msgs::Pose pose_Container4; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);
      pose_Container4.orientation.x = q[0];
      pose_Container4.orientation.y = q[1];
      pose_Container4.orientation.z = q[2];
      pose_Container4.orientation.w = q[3];
      pose_Container4.position.x = primitive_Container4.dimensions[0]/2 + 0.4;
      pose_Container4.position.y = -0.58 - 0.15;
      pose_Container4.position.z = primitive_Container4.dimensions[2]/2 - 0.30 - 0.30;

      Container4.primitives.push_back(primitive_Container4);
      Container4.primitive_poses.push_back(pose_Container4);
      Container4.operation = Container4.ADD;

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

      // ---------- Collect Whole Robot Body

      std::vector<moveit_msgs::CollisionObject> collision_robot_body;
      collision_robot_body.push_back(UGV_base);
      collision_robot_body.push_back(UGV_base2);
      collision_robot_body.push_back(magnet_panel);
      collision_robot_body.push_back(Container1);
      collision_robot_body.push_back(Container2);
      // collision_robot_body.push_back(Container3);
      collision_robot_body.push_back(Container4);
      collision_robot_body.push_back(ground);
      planning_scene_interface.addCollisionObjects(collision_robot_body); //add the collision object into the world
      ros::Duration(DELAY).sleep(); // wait to build the object before attaching to ee
      move_group.attachObject(magnet_panel.id); // attach the magnet panel to end-effector


  }

  void attachBrick(uint color)
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

};  // end of class definition

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbzirc_ur5_control");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  Arm mbzircArm;
  ros::waitForShutdown();

  return 0;
}
