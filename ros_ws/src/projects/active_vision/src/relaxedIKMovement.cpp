// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_planner/GetPose.h>
// #include <moveit_planner/MoveCart.h>
// #include <moveit_planner/MovePose.h>
// #include <moveit_planner/Inv.h>
// #include <moveit_planner/SetVelocity.h>
// #include <moveit_planner/AddCollision.h>
// #include <moveit_planner/SetConstraints.h>
// #include <moveit_planner/MoveNamedState.h>
// #include <moveit_planner/SetJointWithTime.h>
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "move_franka");
//   ros::NodeHandle nh;
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   // Replace "panda_arm" with your robot's planning group name if different
//   static const std::string PLANNING_GROUP = "arm";
//   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

//   // Specify your target joint values
//   // Make sure these names match those in your robot's URDF and MoveIt! configuration
//   std::map<std::string, double> target_joint_values;
//   target_joint_values["panda_joint1"] = 0.0; // radians
//   target_joint_values["panda_joint2"] = 0.0;
//   target_joint_values["panda_joint3"] = 0.0;
//   target_joint_values["panda_joint4"] = -1.5708;
//   target_joint_values["panda_joint5"] = 0.0;
//   target_joint_values["panda_joint6"] = 1.5708;
//   target_joint_values["panda_joint7"] = 0.0;
//   // Continue for all the joints you want to set
  
//   // Set the joint target
//   move_group.setJointValueTarget(target_joint_values);

//   // Optionally, configure the planner
//   move_group.setPlanningTime(10.0); // Set a planning time limit
//   move_group.setNumPlanningAttempts(10); // Set the number of planning attempts
//   move_group.setPlannerId("RRTConnectkConfigDefault"); // Specify the planner

//   // Move the robot to the target
// //   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// //   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// //   if(success) {
// //     move_group.move();
// //   } else {
// //     ROS_WARN("Failed to plan to the specified joint state.");
// //   }

//   // std::map<std::string, double> target_joint_values;
//   // target_joint_values["panda_joint1"] = 0.0; // radians
//   // target_joint_values["panda_joint2"] = 0.0;
//   // target_joint_values["panda_joint3"] = 0.0;
//   // target_joint_values["panda_joint4"] = -1.5708;
//   // target_joint_values["panda_joint5"] = 0.0;
//   // target_joint_values["panda_joint6"] = 1.5708;
//   // target_joint_values["panda_joint7"] = 0.0;
//   // moveit_planner::MoveNamedState moveState;
//   // moveState.request.val = target_joint_values;
//   // moveState.request.request.execute = true;
//   // ros::shutdown();
//   // return 0;
// }
