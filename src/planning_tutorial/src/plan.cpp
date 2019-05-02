#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "sia5";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y += 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y -= 0.25;
  target_pose3.position.x -= 0.25;
  waypoints.push_back(target_pose3);  // up and left

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
  
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;

  
  move_group.execute(plan);
  

  ros::shutdown();
  return 0;
}

