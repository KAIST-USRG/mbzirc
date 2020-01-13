#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>

// #define DEBUG
#define DELAY 1.0         // for sleep function => robot updating states => 0.4 s fail (?)
#define END_EFFECTOR 0.07
#define PLANNING_TIMEOUT 20
#define Z_OFFSET 0.105
#define NUM_SUM 10 //20     // to average the pose msg
#define NUM_DISCARD 5 // 10

namespace rvt = rviz_visual_tools;

// for rotation of objects
tf2::Quaternion q;

// FLAG for robot motion
bool FLAG_AT_DEFAULT = false;   // Is the arm at default position?

bool FLAG_FINISH_PICK = false;  // Has the arm finished picking the brick? (brick attaches to magnet)
bool FLAG_STORED = false;       // Is the brick already stored in on the UGV?

bool FLAG_FINISH_STORING1 = true; // Is the arm storing the first object?
bool FLAG_FINISH_STORING2 = true; // Is the arm storing the second object?
bool FLAG_FINISH_UNLOAD = true;  // Is the arm unloading the object?

bool FLAG_READ_CAM_DATA = true;    // Should the arm read msg from camera?
bool FLAG_MAGNET_ON = true;      // Is the magnet on?


int gb_count_pose_msg = 0;
int gb_discard_noise = 0;
int gb_count_box = 0;
float gb_x_sum = 0;
float gb_y_sum = 0;
float gb_z_sum = 0;
float gb_yaw_sum = 0;


// =============================== call back function ==================================== //
class Arm{
private:
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

  // for checking success of planning
  bool success;

  // for Cartesian Path
  const double jump_threshold = 0.0; // read only.
  const double eef_step = 0.01;      // read only.
  double PI = 3.141592653589793;
  double fraction;

  ros::Subscriber pose_msg_from_cam;
  ros::Subscriber pick_brick;
  ros::Subscriber store_brick;
  ros::Subscriber unload_brick;
  ros::Subscriber magnet_state;

  ros::Publisher magnet_pub;
  ros::Publisher pick_brick_pub;
  ros::Publisher store_brick_pub;
  ros::Publisher unload_brick_pub;

  ros::NodeHandle nh_;
  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroupInterface move_group;

  moveit_visual_tools::MoveItVisualTools visual_tools;
  moveit::core::RobotStatePtr current_state;
  const robot_state::JointModelGroup* joint_model_group;
  Eigen::Isometry3d text_pose;

  std_msgs::Bool magnet_msg;
  std_msgs::Bool pick_brick_msg;
  std_msgs::UInt32 store_brick_msg;
  geometry_msgs::Pose unload_brick_msg; // absolute position w.r.t UR5 base

public:

  Arm():nh_("~"), move_group("manipulator"), visual_tools("base_link") {

    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    current_state = move_group.getCurrentState();

    magnet_msg.data = true;
    pick_brick_msg.data = true;

    joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    text_pose = Eigen::Isometry3d::Identity();

    initWall();
    moveToDefault(true);  // Start at the default position

    magnet_pub = nh.advertise<std_msgs::Bool>("/magnet_on", 1000);
    pick_brick_pub = nh.advertise<std_msgs::Bool>("/pick_brick", 1000);
    store_brick_pub = nh.advertise<std_msgs::UInt32>("/store_brick", 1000);

    // magnet_state = nh.subscribe("/magnet_on", 1000, &Arm::magnetStateCallBack, this);
    // pick_brick = nh.subscribe("/pick_brick", 1000, &Arm::pickBrickCallBack, this);
    pose_msg_from_cam = nh.subscribe("/brick_pose", 1000, &Arm::pickAtCallback, this);
    // store_brick = nh.subscribe("/store_brick", 1000, &Arm::storeBrickCallBack, this);
    // unload_brick = nh.subscribe("/unload_brick", 1000, &Arm::unLoadCallBack, this);
  }

  void magnetStateCallBack(const std_msgs::Bool::ConstPtr& msg){
    // keep track of the magnet state => allows arm to read data only when the magnet is on
    if (msg->data == true){
      FLAG_MAGNET_ON = true;
    }
    else if (msg->data == false){
      FLAG_MAGNET_ON = false;
    }
  }

  void pickBrickCallBack(const std_msgs::Bool::ConstPtr& msg){
    // trigger the arm to start reading pose of the bricks from the camera
    if (msg->data == true){
      FLAG_READ_CAM_DATA = true;
    }
    else if (msg->data == false){
      FLAG_READ_CAM_DATA = false;
    }
  }

  void pickAtCallback(const geometry_msgs::Pose::ConstPtr& msg){
    // read the brick pose from the camera
    // then go and get it

    // z_offset 4 cm, y offset 3 cm
    if (FLAG_READ_CAM_DATA == true && FLAG_MAGNET_ON == true){
      // filtering
      if (FLAG_AT_DEFAULT == true && gb_count_pose_msg < NUM_SUM){
        if (gb_count_pose_msg <= NUM_DISCARD){
          gb_x_sum = 0;
          gb_y_sum = 0;
          gb_z_sum = 0;
          gb_yaw_sum = 0;
        }
        ROS_INFO("accum the data");
        gb_x_sum += msg->position.x;
        gb_y_sum += msg->position.y;
        gb_z_sum += msg->position.z;
        
        tf::Quaternion q(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w
        );

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);
        
        if (yaw<0)
          yaw += PI;
        if (yaw>PI/2)
          yaw = -(PI-yaw);
        gb_yaw_sum += yaw;
        gb_count_pose_msg += 1;
      }else if(FLAG_AT_DEFAULT == true &&
                FLAG_FINISH_UNLOAD == true && FLAG_FINISH_STORING1 == true &&
                FLAG_FINISH_STORING2 == true){ // picking up should start from DEFAULT position
        int n = NUM_SUM - NUM_DISCARD;
        ROS_INFO("x = %lf, y = %lf, z =%lf", gb_x_sum/n, gb_y_sum/n, gb_z_sum/n);
        FLAG_AT_DEFAULT = false;
        #ifdef DEBUG
          visual_tools.prompt("Press 'next' to go to get bricks");
        #endif
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        // Next get the current set of joint values for the group.
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
        // rotate left 90 Deg to store bricks
        joint_group_positions[5] += gb_yaw_sum/n;  // radians
        ROS_INFO("yaw = %f", gb_yaw_sum/n*180./PI);
        move_group.setJointValueTarget(joint_group_positions);
        move_group.setPlanningTime(PLANNING_TIMEOUT);
        move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");

        // Visualize the plan in RViz
        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

        #ifdef DEBUG
            visual_tools.prompt("Press 'next' to front position");
        #endif

        move_group.move(); //move to storage on left side
        ros::Duration(2*DELAY).sleep();           // wait for robot to update current state otherwise failed




        moveFromCurrentState(gb_x_sum/n, gb_y_sum/n, gb_z_sum/n, true);
        gb_count_box += 1;

        gb_count_pose_msg = 0;
        gb_x_sum = 0;
        gb_y_sum = 0;
        gb_z_sum = 0;
        gb_yaw_sum = 0;
        magnet_msg.data = true;
        magnet_pub.publish(magnet_msg); // MAGNET ON
        ROS_INFO("MAGNET_ON");
        ros::Duration(1.).sleep();
        magnet_msg.data = true;
        magnet_pub.publish(magnet_msg); // MAGNET ON
        ROS_INFO("MAGNET_ON");
        ros::Duration(1.).sleep();
        // moveToDefault(true);
        moveToFront();
        FLAG_READ_CAM_DATA = false;
        // finish picking up => call the store
        store_brick_msg.data = gb_count_box;
        store_brick_pub.publish(store_brick_msg); // store the brick on UGV
        ROS_INFO("StoreBrickCallBack");
        storeOnUGV(7);
      }
    }else{
      // don't get the stream data from camera and clear garbage data
      gb_count_pose_msg = 0;
      gb_x_sum = 0;
      gb_y_sum = 0;
      gb_z_sum = 0;
    }
  }

  void storeBrickCallBack(const std_msgs::UInt32::ConstPtr& msg){
      // store the brick on UGV
      pick_brick_msg.data = false;
      pick_brick_pub.publish(pick_brick_msg); // Stop reading camera msg until we turn it on again

      ROS_INFO("StoreBrickCallBack");
      storeOnUGV(msg->data);
      // moveTo()
  }

  void unLoadCallBack(const geometry_msgs::Pose::ConstPtr& msg){
      // unLoad the box according to
      if (FLAG_FINISH_UNLOAD == true){
          FLAG_FINISH_UNLOAD = false;  // should set this FLAG when unloading
          unloadTo(msg->position.x,msg->position.y,msg->position.z, gb_count_box);
          gb_count_box -= 1;
          FLAG_FINISH_UNLOAD = true;   // should set this FLAG when finishing unloading
      }
  }

  void initWall() {

      // Seong) Set planner, Max velo and Planning time
      move_group.setPlannerId("RRTConnectkConfigDefault");
      move_group.setMaxVelocityScalingFactor(0.1);
      move_group.setPlanningTime(PLANNING_TIMEOUT);

      // class to add and remove collision objects in our "virtual world" scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // Raw pointers are frequently used to refer to the planning group for improved performance.
      // ================================== Visualization ======================================= //
      // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
      // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script

      visual_tools.deleteAllMarkers();

      // Remote control is an introspection tool that allows users to step through a high level script
      // via buttons and keyboard shortcuts in RViz
      visual_tools.loadRemoteControl();

      // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres

      text_pose.translation().z() = 1.75;
      visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

      // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
      visual_tools.trigger();

      // ============================== Set Robot Body Collision ============================== //

      // ---------- UGV_base + Side storage area
      moveit_msgs::CollisionObject UGV_base; // Define a collision object ROS message.
      UGV_base.header.frame_id = move_group.getPlanningFrame();

      UGV_base.id = "UGV_base"; // The id of the object is used to identify it.

      shape_msgs::SolidPrimitive primitive_UGV_base; // Define UGV_base dimension (in meter)
      primitive_UGV_base.type = primitive_UGV_base.BOX;
      primitive_UGV_base.dimensions.resize(3);
      primitive_UGV_base.dimensions[0] = 0.66;  // x right
      primitive_UGV_base.dimensions[1] = 0.55;  // y front
      primitive_UGV_base.dimensions[2] = 0.26;  // z up

      geometry_msgs::Pose pose_UGV_base; // Define a pose for the Robot_bottom (specified relative to frame_id)
      q.setRPY(0, 0, 0);      // 90 deg
      pose_UGV_base.orientation.x = q[0];
      pose_UGV_base.orientation.y = q[1];
      pose_UGV_base.orientation.z = q[2];
      pose_UGV_base.orientation.w = q[3];
      pose_UGV_base.position.x =  -0.09;
      pose_UGV_base.position.y = -0.06;
      pose_UGV_base.position.z = -primitive_UGV_base.dimensions[2]/2;

      UGV_base.primitives.push_back(primitive_UGV_base);
      UGV_base.primitive_poses.push_back(pose_UGV_base);
      UGV_base.operation = UGV_base.ADD;

      // ---------- Magnet Panel at end effector
      // moveit_msgs::CollisionObject magnet_panel; // Define a collision object ROS message.
      // magnet_panel.header.frame_id = move_group.getEndEffectorLink(); // reference to end-effector frame
      // magnet_panel.id = "magnet_panel"; // The id of the object is used to identify it.

      // shape_msgs::SolidPrimitive primitive_magnet_panel; // Define UGV_body dimension (in meter)
      // primitive_magnet_panel.type = primitive_magnet_panel.BOX;
      // primitive_magnet_panel.dimensions.resize(3);
      // primitive_magnet_panel.dimensions[0] = 0.035;  // length (x)
      // primitive_magnet_panel.dimensions[1] = 0.42;  // width  (y)
      // primitive_magnet_panel.dimensions[2] = 0.065;  // height (z)


      // geometry_msgs::Pose pose_magnet_panel; // Define a pose for the UGV_body (specified relative to frame_id)
      // q.setRPY(0, 0, 0);
      // pose_magnet_panel.orientation.x = q[0];
      // pose_magnet_panel.orientation.y = q[1];
      // pose_magnet_panel.orientation.z = q[2];
      // pose_magnet_panel.orientation.w = q[3];
      // pose_magnet_panel.position.x = primitive_magnet_panel.dimensions[0]/2;
      // pose_magnet_panel.position.y = 0;
      // pose_magnet_panel.position.z = 0;

      // magnet_panel.primitives.push_back(primitive_magnet_panel);
      // magnet_panel.primitive_poses.push_back(pose_magnet_panel);
      // magnet_panel.operation = magnet_panel.ADD;

      // ---------- Collect Whole Robot Body
      std::vector<moveit_msgs::CollisionObject> collision_robot_body;
      // collision_robot_body.push_back(magnet_panel);
      collision_robot_body.push_back(UGV_base);
      planning_scene_interface.addCollisionObjects(collision_robot_body); //add the collision object into the world
      ros::Duration(DELAY).sleep(); // wait to build the object before attaching to ee

      // move_group.attachObject(magnet_panel.id); // attach the magnet panel to end-effector
  }

  void moveToFront(){

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = PI/2;  // radians
    joint_group_positions[1] = -PI/2;  // radians
    joint_group_positions[2] = PI/4;  // radians
    joint_group_positions[3] = 315.*PI/180.;  // radians
    joint_group_positions[4] = -PI/2;  // radians
    joint_group_positions[5] = PI/4.;  // radians
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setMaxVelocityScalingFactor(0.1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

  #ifdef DEBUG
    visual_tools.prompt("Press 'next' to front position");
  #endif

    move_group.move();                      // move to default position, arm in front of the robot
    ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed

    FLAG_AT_DEFAULT = false;
    };

  void moveToDefault(bool setFlag){
    if (FLAG_AT_DEFAULT == false){
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
      //
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      moveToFront();

      robot_state::RobotState init_state(*move_group.getCurrentState()); // save init state
      geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

      // ========== Cartesian Paths up ========== //

    //   std::vector<geometry_msgs::Pose> waypoints_up;
    //   target_pose3 = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    //   waypoints_up.push_back(target_pose3);

    //   target_pose3.position.z += 0.28;        // up to default position => the highest we can go ~140 cm
    //   waypoints_up.push_back(target_pose3);   // back to the position before going down

    //   move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower

    //   // We want the Cartesian path to be interpolated at a resolution of 1 cm
    //   moveit_msgs::RobotTrajectory trajectory_up;

    //   fraction = move_group.computeCartesianPath(waypoints_up, eef_step, jump_threshold, trajectory_up);
    //   ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath up (%.2f%% acheived)", fraction * 100.0);

    //   cartesian_plan.trajectory_ = trajectory_up;

    //   // Visualize the plan in RViz
    //   visual_tools.deleteAllMarkers();
    //   visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    //   visual_tools.publishPath(waypoints_up, rvt::LIME_GREEN, rvt::SMALL);
    //   for (std::size_t i = 0; i < waypoints_up.size(); ++i)
    //     visual_tools.publishAxisLabeled(waypoints_up[i], "pt" + std::to_string(i), rvt::SMALL);
    //   visual_tools.trigger();

    #ifdef DEBUG
      visual_tools.prompt("Press 'next' to go to default position");
    #endif
      move_group.execute(cartesian_plan);
      if (setFlag == true){
        FLAG_AT_DEFAULT = true;
      }
      ros::Duration(3*DELAY).sleep(); //sleep to wait for stable msg from camera

    }else{
      // do nothing if it's already at the default position
      ROS_INFO("moveToDefault: already at the default position");
    }

  };

  void moveFromCurrentState(float toX, float toY, float toZ, bool isPicking){

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_down;
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    waypoints_down.push_back(target_pose);

    if (isPicking == true){
      target_pose.position.x += (toX);                  // #######################################################################################################
      target_pose.position.y += (toY-0.03);           // #######################################################################################################
      target_pose.position.z += (-1.07+0.05); // 0.05 // + up  // #######################################################################################################
      waypoints_down.push_back(target_pose);    // back to the position before going down
    }else{
      target_pose.position.x += toX; // + right
      target_pose.position.y += toY; // + front
      target_pose.position.z += toZ; // + up
      waypoints_down.push_back(target_pose);    // back to the position before going down
    }

    move_group.setMaxVelocityScalingFactor(0.07); // Cartesian motions are needed to be slower

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    moveit_msgs::RobotTrajectory trajectory_down;

    fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
    ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath down (%.2f%% acheived)", fraction * 100.0);

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
    ros::Duration(DELAY).sleep();
  #endif

    move_group.execute(cartesian_plan);
    ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed

    if (isPicking == true){     // to inform that robot has done picking job
      FLAG_FINISH_PICK = true;
      ros::Duration(2*DELAY).sleep();
    }else{
      FLAG_FINISH_PICK = false;
    }

    FLAG_AT_DEFAULT = false;
  };

  void moveTo(float toX, float toY, float toZ){

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_To;
    target_pose = move_group.getCurrentPose().pose; // Cartesian Path from the current position
    waypoints_To.push_back(target_pose);

    target_pose.position.x = toX; // + right
    target_pose.position.y = toY; // + front
    target_pose.position.z = toZ; // + up

    waypoints_To.push_back(target_pose);    // back to the position before going down

    move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    moveit_msgs::RobotTrajectory trajectory_down;

    fraction = move_group.computeCartesianPath(waypoints_To, eef_step, jump_threshold, trajectory_down);
    ROS_INFO_NAMED("tutorial", "Visualizing CartesianPath down (%.2f%% acheived)", fraction * 100.0);

    cartesian_plan.trajectory_ = trajectory_down;

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints_To, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints_To.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints_To[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

  #ifdef DEBUG
    visual_tools.prompt("Press 'next' to go down");
    ros::Duration(DELAY).sleep();
  #endif

    move_group.execute(cartesian_plan);
    ros::Duration(2*DELAY).sleep();           // wait for robot to update current state otherwise failed

    FLAG_AT_DEFAULT = false;
  };

  void moveToStorage(){

    moveToDefault(false);
    // assume we are at default position
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    // rotate left 90 Deg to store bricks
    joint_group_positions[0] = PI;  // radians
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    #ifdef DEBUG
        visual_tools.prompt("Press 'next' to front position");
    #endif

    move_group.move(); //move to storage on left side
    ros::Duration(2*DELAY).sleep();           // wait for robot to update current state otherwise failed


    FLAG_AT_DEFAULT = false;
  };

  void storeOnUGV(int count){
    ROS_INFO("Storing: count = %d", count);
    FLAG_STORED = true;
    // moveToStorage();

    // geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    // target_pose = move_group.getCurrentPose().pose;
    // if (count == 1){
    //   moveTo(-0.35, target_pose.position.y, 0.345); // bring arm backward and down      // #######################################################################################################
    // }else if (count == 2){
    //   moveTo(-0.35, target_pose.position.y, 0.345 + 0.22); // bring arm backward and down // #######################################################################################################
    // }
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -PI/2;  // radians
    joint_group_positions[1] = -120.*PI/180.;  // radians
    joint_group_positions[2] = 120.*PI/180.;  // radians
    joint_group_positions[3] = 270.*PI/180.;  // radians
    joint_group_positions[4] = -PI/2;  // radians
    joint_group_positions[5] = PI/4.;  // radians
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlanningTime(PLANNING_TIMEOUT);
    move_group.setMaxVelocityScalingFactor(0.3);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

  #ifdef DEBUG
    visual_tools.prompt("Press 'next' to front position");
  #endif

    move_group.move();                      // move to default position, arm in front of the robot
    ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed

    FLAG_AT_DEFAULT = false;

    
    magnet_msg.data = false;
    magnet_pub.publish(magnet_msg); // MAGNET OFF
    ROS_INFO("MAGNET_OFF");
    ros::Duration(3.).sleep();           // wait for robot to update current state otherwise failed
    // go back to default position after finishing storing the bricks
    moveToDefault(true);

    // magnet_msg.data = true;
    // magnet_pub.publish(magnet_msg); // MAGNET ON
    // ROS_INFO("MAGNET_ON");

    FLAG_STORED = false;
    FLAG_FINISH_PICK = false;
  };

  void unloadTo(float atX, float atY, float atZ, int count){
    ROS_INFO("unloadTo");

    // unload to absolute coordinate w.r.t. base-frame
    magnet_msg.data = true;
    magnet_pub.publish(magnet_msg); // MAGNET ON

    moveToStorage();
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    target_pose = move_group.getCurrentPose().pose;

    // picking up from storage
    if (count == 1){
      moveTo(-0.35, target_pose.position.y, 0.342 + 0.1); // #######################################################################################################
    }else if(count == 2){
      moveTo(-0.35, target_pose.position.y, 0.342 + 0.20); // #######################################################################################################
    }

    // placing down at X, Y, Z w.r.t. to base-frame
    moveToFront();
    moveTo(atX, atY, (atZ + 0.345)); //0.345 = brick height + magnetic // #######################################################################################################
    ROS_INFO("unloadTo: Move to predefine position");

    magnet_msg.data = false;
    magnet_pub.publish(magnet_msg); // MAGNET OFF
    ROS_INFO("unloadTo: MAGNET OFF");
    // go back to default position
    moveToDefault(true);
    ROS_INFO("unloadTo: Move to Default");
    magnet_msg.data = true;
    magnet_pub.publish(magnet_msg); // MAGNET ON
    ROS_INFO("unloadTo: MAGNET ON");

  }

};  // end of class def

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Arm mbzircArm;
  ros::waitForShutdown();

  return 0;
}