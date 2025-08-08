#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>   // <-- AsyncParametersClient ici

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <map>
#include <vector>
#include <string>
#include <future>
#include <chrono>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rosie_dual_arms_cpp_demo");

// ---------- Helpers ----------
inline bool ok(const moveit::core::MoveItErrorCode& c) {
  return c == moveit::core::MoveItErrorCode::SUCCESS;
}

inline const char* to_cstr(const moveit::core::MoveItErrorCode& c) {
  using MEC = moveit::core::MoveItErrorCode;
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

// Utilitaire pour récupérer un paramètre string depuis un AsyncParametersClient
static bool get_remote_param_string(
    rclcpp::AsyncParametersClient& client,
    const std::string& name,
    std::string& out)
{
  auto fut = client.get_parameters({name});
  if (fut.wait_for(5s) != std::future_status::ready) {
    return false;
  }
  auto params = fut.get();
  if (params.empty()) {
    return false;
  }
  try {
    out = params.front().get_value<std::string>();
    return true;
  } catch (...) {
    return false;
  }
}

// Importe (au minimum) robot_description & robot_description_semantic (+ kinematics si dispo)
static void import_moveit_params_into_node(
    const rclcpp::Node::SharedPtr& node,
    const std::string& src = "/move_group")
{
  using namespace std::chrono_literals;

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, src);

  RCLCPP_INFO(LOGGER, "Waiting for %s parameters service...", src.c_str());
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok())
      throw std::runtime_error("Interrupted while waiting for /move_group parameter service");
    RCLCPP_INFO(LOGGER, "Still waiting for %s...", src.c_str());
  }

  // --- 1) Importer les "gros" paramètres de base (URDF, SRDF, etc.) ---
  const std::vector<std::string> core_names = {
      "robot_description",
      "robot_description_semantic",
      "moveit_controller_manager",
      "moveit_controller_names",
      "planning_pipelines"
  };

  auto fut = client->get_parameters(core_names);
  if (fut.wait_for(5s) != std::future_status::ready)
    throw std::runtime_error("Timeout getting parameters from " + src);
  const auto core_params = fut.get();

  std::vector<rclcpp::Parameter> to_set;
  to_set.reserve(core_params.size());

  for (const auto& p : core_params)
  {
    if (p.get_name().empty() || p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      continue;

    if (!node->has_parameter(p.get_name()))
      node->declare_parameter(p.get_name(), p.get_parameter_value());

    to_set.push_back(p);
    RCLCPP_INFO(LOGGER, "Imported '%s' (type=%d)", p.get_name().c_str(), static_cast<int>(p.get_type()));
  }

  node->set_parameters(to_set);

  // --- 2) Importer TOUT le namespace robot_description_kinematics.* ---
  {
    auto list_future = client->list_parameters({"robot_description_kinematics"}, 10 /* depth */);
    if (list_future.wait_for(5s) == std::future_status::ready) {
      const auto lp = list_future.get();
      if (!lp.names.empty()) {
        auto kin_future = client->get_parameters(lp.names);
        if (kin_future.wait_for(5s) == std::future_status::ready) {
          const auto kin_params = kin_future.get();
          for (const auto& p : kin_params) {
          if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
            continue;

          if (!node->has_parameter(p.get_name()))
            node->declare_parameter(p.get_name(), p.get_parameter_value());

          node->set_parameter(p);
          RCLCPP_INFO(LOGGER, "Imported kinematics param '%s' (type=%d)",
                      p.get_name().c_str(), static_cast<int>(p.get_type()));
          }
        } else {
          RCLCPP_WARN(LOGGER, "Timeout while getting robot_description_kinematics.* parameters");
        }
      } else {
        RCLCPP_WARN(LOGGER, "No robot_description_kinematics.* parameters found on %s", src.c_str());
      }
    } else {
      RCLCPP_WARN(LOGGER, "Timeout while listing robot_description_kinematics.* parameters");
    }
  }

  // Sanity check minimal
  if (!node->has_parameter("robot_description") ||
      !node->has_parameter("robot_description_semantic"))
  {
    throw std::runtime_error("robot_description or robot_description_semantic missing after import");
  }

  RCLCPP_INFO(LOGGER, "MoveIt parameters successfully imported on this node.");
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

  auto node = rclcpp::Node::make_shared(
      "rosie_dual_arms_cpp_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  try
  {
    // 0) Importer les params depuis /move_group
    import_moveit_params_into_node(node);

    // 1) Interfaces MoveGroup
    moveit::planning_interface::MoveGroupInterface left_arm(node, "left_arm");
    moveit::planning_interface::MoveGroupInterface right_arm(node, "right_arm");
    moveit::planning_interface::MoveGroupInterface left_gripper(node, "left_gripper");
    moveit::planning_interface::MoveGroupInterface right_gripper(node, "right_gripper");
    moveit::planning_interface::MoveGroupInterface both_arms(node, "both_arms");


    RCLCPP_INFO(LOGGER, "Planning with groups: %s, %s",
                left_arm.getName().c_str(), right_arm.getName().c_str());

    // 2) Paramètres globaux
    left_arm.setPlanningPipelineId("ompl");
    right_arm.setPlanningPipelineId("ompl");
    both_arms.setPlanningPipelineId("ompl");

    left_arm.setPlannerId("RRTConnectkConfigDefault");
    right_arm.setPlannerId("RRTConnectkConfigDefault");
    both_arms.setPlannerId("RRTConnectkConfigDefault");

    left_arm.setNumPlanningAttempts(5);
    right_arm.setNumPlanningAttempts(5);
    both_arms.setNumPlanningAttempts(5);
    left_arm.setPlanningTime(5.0);
    right_arm.setPlanningTime(5.0);
    both_arms.setPlanningTime(5.0);

    left_arm.setStartStateToCurrentState();
    right_arm.setStartStateToCurrentState();
    both_arms.setStartStateToCurrentState();

    bool use_named_targets = false;

    if (use_named_targets)
    {
      left_arm.setNamedTarget("front_home");
      right_arm.setNamedTarget("front_home");

      auto rc_l = left_arm.move();
      auto rc_r = right_arm.move();
      RCLCPP_INFO(LOGGER, "Left arm to home: %s (%d)", ok(rc_l) ? "OK" : "FAIL", rc_l.val);
      RCLCPP_INFO(LOGGER, "Right arm to home: %s (%d)", ok(rc_r) ? "OK" : "FAIL", rc_r.val);
    }
    else
    {
      // 4) Pose target sur le bras gauche
      const std::string left_ee_link = left_arm.getEndEffectorLink();
      const std::string left_frame   = left_arm.getPlanningFrame();

      RCLCPP_INFO(LOGGER, "Left EE link: %s", left_ee_link.c_str());
      RCLCPP_INFO(LOGGER, "Left planning frame: %s", left_frame.c_str());

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "arm_world";
      target_pose.pose.position.x = 0.0;
      target_pose.pose.position.y = 0.4;
      target_pose.pose.position.z = 0.5;
      target_pose.pose.orientation.w = 1.0;

      left_arm.setPoseTarget(target_pose, left_ee_link);

      moveit::planning_interface::MoveGroupInterface::Plan left_plan;
      auto rc_plan = left_arm.plan(left_plan);
      RCLCPP_INFO(LOGGER, "Left arm plan result: %s (%d)", to_cstr(rc_plan), rc_plan.val);

      if (ok(rc_plan))
      {
        RCLCPP_INFO(LOGGER, "Executing left arm plan...");
        auto rc_exec = left_arm.execute(left_plan);
        RCLCPP_INFO(LOGGER, "Left arm execution: %s (%d)", to_cstr(rc_exec), rc_exec.val);
      }
      else
      {
        RCLCPP_WARN(LOGGER, "Left arm planning failed");
      }

      // Récupérer l'état courant des 14 joints
      auto current_state = *both_arms.getCurrentState();
      std::vector<double> q_both;
      current_state.copyJointGroupPositions("both_arms", q_both);

      // Sanity check
      if (q_both.size() != 14) {
        RCLCPP_ERROR(LOGGER, "[both_arms] attendu 14 valeurs (2x7 ddl); reçu %zu. Vérifie ton SRDF.", q_both.size());
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

      // 5) Pince gauche : commande GripperCommand (un seul joint maître)
      std::map<std::string, double> target_gripper;
      target_gripper["leftrobotiq_85_left_knuckle_joint"] = 0.8;
      left_gripper.setJointValueTarget(target_gripper);

      auto rc_g = left_gripper.move();
      RCLCPP_INFO(LOGGER, "Left gripper close: %s (%d)", ok(rc_g) ? "OK" : "FAIL", rc_g.val);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(LOGGER, "Fatal error: %s", e.what());
  }

  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}

