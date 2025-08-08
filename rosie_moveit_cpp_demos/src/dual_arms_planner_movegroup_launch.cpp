#include <rclcpp/rclcpp.hpp>
//#include <moveit/move_group_interface/move_group_interface.hpp> //Moveit2 main branch
#include <moveit/move_group_interface/move_group_interface.h> //Moveit2 humble branch
//#include <moveit/planning_scene_interface/planning_scene_interface.hpp> //Moveit2 main branch
#include <moveit/planning_scene_interface/planning_scene_interface.h> //Moveit2 humble branch
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>                    
#include <map>

// Useful Alias
using MvErr = moveit::core::MoveItErrorCode;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rosie_dual_arms_cpp_demo");

// ---------- Helpers ----------
inline bool ok(const MvErr& c) {
  return c == MvErr::SUCCESS;
}

inline const char* to_cstr(const MvErr& c) {
  using MEC = MvErr;
  switch (c.val) {
    case MEC::SUCCESS: return "SUCCESS";
    case MEC::PLANNING_FAILED: return "PLANNING_FAILED";
    case MEC::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case MEC::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: return "PLAN_INVALIDATED_BY_ENV_CHANGE";
    case MEC::CONTROL_FAILED: return "CONTROL_FAILED";
    case MEC::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_ACQUIRE_SENSOR_DATA";
    case MEC::TIMED_OUT: return "TIMED_OUT";
    case MEC::PREEMPTED: return "PREEMPTED";
    case MEC::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case MEC::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case MEC::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case MEC::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MEC::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case MEC::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case MEC::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case MEC::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case MEC::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case MEC::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    default: return "UNKNOWN";
  }
}

bool plan_and_execute_joints(moveit::planning_interface::MoveGroupInterface& mgi,
                             const std::vector<double>& joints)
{
  mgi.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto rc = mgi.plan(plan);
  RCLCPP_INFO(LOGGER, "[%s] joint-space plan: %s (%d)", mgi.getName().c_str(), to_cstr(rc), rc.val);
  if (!ok(rc)) return false;
  auto rc_exec = mgi.execute(plan);
  RCLCPP_INFO(LOGGER, "[%s] exec: %s (%d)", mgi.getName().c_str(), to_cstr(rc_exec), rc_exec.val);
  return ok(rc_exec);
}

// ---------- main ----------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // User node
  auto node = rclcpp::Node::make_shared("rosie_dual_arms_cpp_demo");

  // Executor + thread spin useful for MoveGroupInterface
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // === 1) MoveGroup Interfaces setup ===
  moveit::planning_interface::MoveGroupInterface left_arm(node, "left_arm");
  moveit::planning_interface::MoveGroupInterface right_arm(node, "right_arm");
  moveit::planning_interface::MoveGroupInterface left_gripper(node, "left_gripper");
  moveit::planning_interface::MoveGroupInterface right_gripper(node, "right_gripper");
  moveit::planning_interface::MoveGroupInterface both_arms(node, "both_arms");

  RCLCPP_INFO(LOGGER, "Planning with groups: %s, %s, %s",
              left_arm.getName().c_str(),
              right_arm.getName().c_str(),
              both_arms.getName().c_str());

  // === 2) global setup ===
  left_arm.setPlanningPipelineId("ompl");
  right_arm.setPlanningPipelineId("ompl");
  both_arms.setPlanningPipelineId("ompl");

  left_arm.setPlannerId("RRTConnectkConfigDefault");
  right_arm.setPlannerId("RRTConnectkConfigDefault");
  both_arms.setPlannerId("RRTConnectkConfigDefault");

  left_arm.setMaxVelocityScalingFactor(0.3);  
  left_arm.setMaxAccelerationScalingFactor(0.3);  

  right_arm.setMaxVelocityScalingFactor(0.3);
  right_arm.setMaxAccelerationScalingFactor(0.3);

  both_arms.setMaxVelocityScalingFactor(0.3);
  both_arms.setMaxAccelerationScalingFactor(0.3);

  left_gripper.setMaxVelocityScalingFactor(0.5);
  left_gripper.setMaxAccelerationScalingFactor(0.5);
  right_gripper.setMaxVelocityScalingFactor(0.5);
  right_gripper.setMaxAccelerationScalingFactor(0.5);

  left_arm.setNumPlanningAttempts(10);
  right_arm.setNumPlanningAttempts(10);
  both_arms.setNumPlanningAttempts(10);
  left_arm.setPlanningTime(5.0);
  right_arm.setPlanningTime(5.0);
  both_arms.setPlanningTime(5.0);

  left_arm.setStartStateToCurrentState();
  right_arm.setStartStateToCurrentState();
  both_arms.setStartStateToCurrentState();

  // === 3) named targets ===
  left_arm.setNamedTarget("front_home");
  right_arm.setNamedTarget("front_home");

  bool left_plan_success  = (left_arm.move()  == MvErr::SUCCESS);
  bool right_plan_success = (right_arm.move() == MvErr::SUCCESS);

  RCLCPP_INFO(LOGGER, "Left arm to home: %s",  left_plan_success  ? "OK" : "FAIL");
  RCLCPP_INFO(LOGGER, "Right arm to home: %s", right_plan_success ? "OK" : "FAIL");

  // === 4) Pose target left arm ===
  const std::string left_ee_link = left_arm.getEndEffectorLink();
  const std::string left_frame   = left_arm.getPlanningFrame();
  RCLCPP_INFO(LOGGER, "Left EE link: %s", left_ee_link.c_str());
  RCLCPP_INFO(LOGGER, "Left frame: %s",   left_frame.c_str()); //base_footprint is not the good frame use arm_world for dual-arm or world for one arm 

  geometry_msgs::msg::PoseStamped target_pose_left;
  target_pose_left.header.frame_id = "arm_world";
  target_pose_left.pose.position.x = 0.0;
  target_pose_left.pose.position.y = 0.4;
  target_pose_left.pose.position.z = 0.5;
  target_pose_left.pose.orientation.w = 1.0;

  left_arm.setPoseTarget(target_pose_left, left_ee_link);

  moveit::planning_interface::MoveGroupInterface::Plan left_plan;
  bool success_left = (left_arm.plan(left_plan) == MvErr::SUCCESS);
  if (success_left)
  {
    RCLCPP_INFO(LOGGER, "Executing left arm plan...");
    bool exec_ok = (left_arm.execute(left_plan) == MvErr::SUCCESS);
    RCLCPP_INFO(LOGGER, "Left arm execution: %s", exec_ok ? "OK" : "FAIL");
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Left arm planning failed");
  }
  // === 4) Pose target right arm ===
  const std::string right_ee_link = right_arm.getEndEffectorLink();
  const std::string right_frame   = right_arm.getPlanningFrame();
  RCLCPP_INFO(LOGGER, "right EE link: %s", right_ee_link.c_str());
  RCLCPP_INFO(LOGGER, "right frame: %s",   right_frame.c_str());

  geometry_msgs::msg::PoseStamped target_pose_right;
  target_pose_right.header.frame_id = "arm_world";
  target_pose_right.pose.position.x = 0.0;
  target_pose_right.pose.position.y = -0.4;
  target_pose_right.pose.position.z = 0.5;
  target_pose_right.pose.orientation.w = 1.0;

  right_arm.setPoseTarget(target_pose_right, right_ee_link);

  moveit::planning_interface::MoveGroupInterface::Plan right_plan;
  bool success_right = (right_arm.plan(right_plan) == MvErr::SUCCESS);
  if (success_right)
  {
    RCLCPP_INFO(LOGGER, "Executing left arm plan...");
    bool exec_ok = (right_arm.execute(right_plan) == MvErr::SUCCESS);
    RCLCPP_INFO(LOGGER, "Right arm execution: %s", exec_ok ? "OK" : "FAIL");
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Right arm planning failed");
  }    

  // === 5) Joints Pose for both arm ===
  auto current_state = *both_arms.getCurrentState();
  std::vector<double> q_both;
  current_state.copyJointGroupPositions("both_arms", q_both);

  // Sanity check
  if (q_both.size() != 14) {
    RCLCPP_ERROR(LOGGER, "[both_arms] waitting 14 values (2x7 dof); receveid %zu. see your srdf.", q_both.size());
  } else {
    q_both[0] = 1.57;   
    q_both[1] = 0.26;   
    q_both[2] = 3.14;   
    q_both[3] = -2.27; 
    q_both[4] = 0.0;   
    q_both[5] = 0.97; 
    q_both[6] = 1.57;   
    q_both[7] = -1.57; 
    q_both[8] = 0.26;   
    q_both[9] = 3.14; 
    q_both[10] = -2.27;   
    q_both[11] = 0.0; 
    q_both[12] = 0.97;   
    q_both[13] = 1.57; 
    both_arms.setStartStateToCurrentState();
    plan_and_execute_joints(both_arms, q_both);
  }

  // === 6) left gripper (joint command) ===
  std::map<std::string, double> target_gripper;
  target_gripper["rightrobotiq_85_left_knuckle_joint"] = 0.8;

  right_gripper.setJointValueTarget(target_gripper);
  bool g_plan_success = (right_gripper.move() == MvErr::SUCCESS);
  RCLCPP_INFO(LOGGER, "right gripper close: %s", g_plan_success ? "OK" : "FAIL");


  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}
