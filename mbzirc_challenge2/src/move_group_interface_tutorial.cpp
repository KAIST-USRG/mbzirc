/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define DEBUG
#define DELAY 1.0         // for sleep function => robot updating states => 0.4 s fail (?)

#define END_EFFECTOR 0.07

#define PLANNING_TIMEOUT 20

// for rotation of objects
tf2::Quaternion q;

// for execute Planning
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

// for checking success of planning
bool success;

// for Cartesian Path
const double jump_threshold = 0.0; // read only.
const double eef_step = 0.01;      // read only.
double PI = 3.141592653589793;
double fraction;

// =============================== call back function ==================================== //

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ===================================== Setup ========================================== //
  static const std::string PLANNING_GROUP = "manipulator";  // UR5 name
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Seong) Set planner, Max velo and Planning time
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setPlanningTime(PLANNING_TIMEOUT);

  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // ================================== Visualization ======================================= //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
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
  primitive_UGV_base.dimensions[1] = 0.42;  // y front
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

  // ---------- Robot Body UGV_body
  moveit_msgs::CollisionObject UGV_body; // Define a collision object ROS message.
  UGV_body.header.frame_id = move_group.getPlanningFrame();

  UGV_body.id = "UGV_body"; // The id of the object is used to identify it.

  shape_msgs::SolidPrimitive primitive_UGV_body;
  primitive_UGV_body.type = primitive_UGV_body.BOX;
  primitive_UGV_body.dimensions.resize(3);
  primitive_UGV_body.dimensions[0] = 0.48;  // x
  primitive_UGV_body.dimensions[1] = 0.50;  // y
  primitive_UGV_body.dimensions[2] = 1.31;  // z

  geometry_msgs::Pose pose_UGV_body;
  q.setRPY(0, 0, 0);
  pose_UGV_body.orientation.x = q[0];
  pose_UGV_body.orientation.y = q[1];
  pose_UGV_body.orientation.z = q[2];
  pose_UGV_body.orientation.w = q[3];
  pose_UGV_body.position.x =  0;
  pose_UGV_body.position.y = -0.52;
  pose_UGV_body.position.z =  0.395;

  UGV_body.primitives.push_back(primitive_UGV_body);
  UGV_body.primitive_poses.push_back(pose_UGV_body);
  UGV_body.operation = UGV_body.ADD;

  // ---------- Magnet Panel at end effector
  moveit_msgs::CollisionObject magnet_panel; // Define a collision object ROS message.
  magnet_panel.header.frame_id = move_group.getEndEffectorLink(); // reference to end-effector frame
  magnet_panel.id = "magnet_panel"; // The id of the object is used to identify it.

  shape_msgs::SolidPrimitive primitive_magnet_panel; // Define UGV_body dimension (in meter)
  primitive_magnet_panel.type = primitive_magnet_panel.BOX;
  primitive_magnet_panel.dimensions.resize(3);
  primitive_magnet_panel.dimensions[0] = 0.035;  // length (x)
  primitive_magnet_panel.dimensions[1] = 0.42;  // width  (y)
  primitive_magnet_panel.dimensions[2] = 0.065;  // height (z)


  geometry_msgs::Pose pose_magnet_panel; // Define a pose for the UGV_body (specified relative to frame_id)
  q.setRPY(0, 0, 0);
  pose_magnet_panel.orientation.x = q[0];
  pose_magnet_panel.orientation.y = q[1];
  pose_magnet_panel.orientation.z = q[2];
  pose_magnet_panel.orientation.w = q[3];
  pose_magnet_panel.position.x = primitive_magnet_panel.dimensions[0]/2;
  pose_magnet_panel.position.y = 0;
  pose_magnet_panel.position.z = 0;

  magnet_panel.primitives.push_back(primitive_magnet_panel);
  magnet_panel.primitive_poses.push_back(pose_magnet_panel);
  magnet_panel.operation = magnet_panel.ADD;

  // ---------- Collect Whole Robot Body
  std::vector<moveit_msgs::CollisionObject> collision_robot_body;
  collision_robot_body.push_back(magnet_panel);
  collision_robot_body.push_back(UGV_base);
  collision_robot_body.push_back(UGV_body);
  planning_scene_interface.addCollisionObjects(collision_robot_body); //add the collision object into the world
  ros::Duration(DELAY).sleep(); // wait to build the object before attaching to ee

  move_group.attachObject(magnet_panel.id); // attach the magnet panel to end-effector


  ros::Subscriber sub = node_handle.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  ////////////////////////////////////////////////////////////////////////////////////////////
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
//  // =========== initial position ===========
//
//  // an pointer that references the current robot's state.
//  // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  // =========== =========== =========== =========== =========== =========== =========== =========== =========== =========== ===========
//  joint_group_positions[0] = PI/2;  // radians
//  joint_group_positions[1] = -PI/2;  // radians
//  joint_group_positions[2] = PI/2;  // radians
//  joint_group_positions[3] = -PI/2;  // radians
//  joint_group_positions[4] = -PI/2;  // radians
//  joint_group_positions[5] = 0;  // radians
//  move_group.setJointValueTarget(joint_group_positions);
//  move_group.setPlanningTime(PLANNING_TIMEOUT);
//
//  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing Initial joint plan (joint space goal) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//
//#ifdef DEBUG
//  visual_tools.prompt("Press 'next' to front position");
//#else
//  ros::Duration(DELAY).sleep();
//#endif
//
//  move_group.move(); //move to default position, arm in front of the robot
//  ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed
//
//  robot_state::RobotState init_state(*move_group.getCurrentState()); // save init state
//  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
//
//  // ========== Cartesian Paths up ========== //
//
//  std::vector<geometry_msgs::Pose> waypoints_up;
//  target_pose3 = move_group.getCurrentPose().pose; // Cartesian Path from the current position
//  waypoints_up.push_back(target_pose3);
//
//  target_pose3.position.z += 0.50;        // up to default position
//  waypoints_up.push_back(target_pose3);    // back to the position before going down
//
//  move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower
//
//  // We want the Cartesian path to be interpolated at a resolution of 1 cm
//  moveit_msgs::RobotTrajectory trajectory_up;
//
//  fraction = move_group.computeCartesianPath(waypoints_up, eef_step, jump_threshold, trajectory_up);
//  ROS_INFO_NAMED("tutorial", "Visualizing Plan2-2 (CartesianPath up) (%.2f%% acheived)", fraction * 100.0);
//
//  cartesian_plan.trajectory_ = trajectory_up;
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishPath(waypoints_up, rvt::LIME_GREEN, rvt::SMALL);
//  for (std::size_t i = 0; i < waypoints_up.size(); ++i)
//    visual_tools.publishAxisLabeled(waypoints_up[i], "pt" + std::to_string(i), rvt::SMALL);
//  visual_tools.trigger();
//
//#ifdef DEBUG
//  visual_tools.prompt("Press 'next' to go to default position");
//#else
//  ros::Duration(DELAY).sleep();
//#endif
//  move_group.execute(cartesian_plan);
//  ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed
//
//  // =========== =========== =========== =========== =========== =========== =========== =========== =========== =========== ===========
//
//
//  // ========== Cartesian Paths Down ========== //
//  std::vector<geometry_msgs::Pose> waypoints_down;
//  target_pose3 = move_group.getCurrentPose().pose; // Cartesian Path from the current position
//  waypoints_down.push_back(target_pose3);
//
//  target_pose3.position.x -= 0.21; // + right
//  target_pose3.position.y += 0.09; // + front
//  target_pose3.position.z -= 0.72; // + up
//  waypoints_down.push_back(target_pose3);    // back to the position before going down
//
//  move_group.setMaxVelocityScalingFactor(0.1); // Cartesian motions are needed to be slower
//
//  // We want the Cartesian path to be interpolated at a resolution of 1 cm
//  moveit_msgs::RobotTrajectory trajectory_down;
//
//  fraction = move_group.computeCartesianPath(waypoints_down, eef_step, jump_threshold, trajectory_down);
//  ROS_INFO_NAMED("tutorial", "Visualizing Plan2-2 (CartesianPath up) (%.2f%% acheived)", fraction * 100.0);
//
//  cartesian_plan.trajectory_ = trajectory_down;
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishPath(waypoints_down, rvt::LIME_GREEN, rvt::SMALL);
//  for (std::size_t i = 0; i < waypoints_down.size(); ++i)
//    visual_tools.publishAxisLabeled(waypoints_down[i], "pt" + std::to_string(i), rvt::SMALL);
//  visual_tools.trigger();
//
//#ifdef DEBUG
//  visual_tools.prompt("Press 'next' to go down");
//#else
//  ros::Duration(DELAY).sleep();
//#endif
//  move_group.execute(cartesian_plan);
//  ros::Duration(DELAY).sleep();           // wait for robot to update current state otherwise failed

  // ========== To store the bricks ========== //

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  ros::shutdown();
  return 0;
}