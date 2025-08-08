#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <chrono>
#include <future>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rosie_moveitcpp_demo");

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
    "planning_pipelines"    // peut être un tableau (ancien style)
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

/* -------------------------------------------------------------------------- */
/*                                     MAIN                                   */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("rosie_moveitcpp_demo", node_options);

  // Executor pour le CurrentStateMonitor, etc.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  try
  {
    /* 0) importer les params MoveIt exposés par /move_group */
    import_moveit_params_into_node(node);

    // Petite pause pour que les TF / joint_states arrivent
    rclcpp::sleep_for(1s);

    /* 1) Instancier MoveItCpp et les PlanningComponent nécessaires */
    moveit_cpp::MoveItCpp::Options opts(node);

    // --- lire la liste des pipelines (nouveau et ancien style) ---
    std::vector<std::string> pipeline_names;   // vide par défaut
    {
      rclcpp::AsyncParametersClient client(node, "/move_group");
      if (client.wait_for_service(2s)) {
        // style récent : planning_pipelines.pipeline_names = [ ... ]
        auto f1 = client.get_parameters({"planning_pipelines.pipeline_names"});
        if (f1.wait_for(1s) == std::future_status::ready) {
          const auto p = f1.get().front();
          if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
            pipeline_names = p.as_string_array();
        }
        // fallback style ancien : planning_pipelines = [ ... ]
        if (pipeline_names.empty()) {
          auto f2 = client.get_parameters({"planning_pipelines"});
          if (f2.wait_for(1s) == std::future_status::ready) {
            const auto p = f2.get().front();
            if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
              pipeline_names = p.as_string_array();
          }
        }
      }
      // S’il n’y avait rien, on force « ompl »
      if (pipeline_names.empty()) {
        RCLCPP_WARN(LOGGER, "planning_pipelines.pipeline_names absent ; on utilise ['ompl']");
        pipeline_names = {"ompl"};
      }
    }

    opts.planning_pipeline_options.pipeline_names = pipeline_names;
    // *** IMPORTANT *** : le parent_namespace doit pointer vers /move_group/planning_pipelines
    opts.planning_pipeline_options.parent_namespace = "/move_group/planning_pipelines";

    auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node, opts);
    moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();

    // Groups
    const std::string LEFT_ARM   = "left_arm";
    const std::string BOTH_ARMS  = "both_arms";

    auto left_pc  = std::make_shared<moveit_cpp::PlanningComponent>(LEFT_ARM,  moveit_cpp);
    auto both_pc  = std::make_shared<moveit_cpp::PlanningComponent>(BOTH_ARMS, moveit_cpp);

    const auto robot_model = moveit_cpp->getRobotModel();
    const auto jmg_left    = robot_model->getJointModelGroup(LEFT_ARM);
    const auto jmg_both    = robot_model->getJointModelGroup(BOTH_ARMS);


    /* ------------------------- ÉTAPE 1: LEFT_ARM en pose ------------------------- */

    left_pc->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "arm_world"; 
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.4;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.w = 1.0;

    // Donne le nom du lien EE du bras gauche
    const std::string ee_link = jmg_left->getLinkModelNames().back(); // ou "leftend_effector_link"
    left_pc->setGoal(target_pose, ee_link);

    moveit_cpp::PlanningComponent::PlanRequestParameters left_params;
    left_params.planning_pipeline = "ompl";
    left_params.planner_id        = "RRTConnectkConfigDefault";
    left_params.planning_time     = 5.0;
    left_params.planning_attempts = 10;

    auto plan_left = left_pc->plan(left_params);
    if (plan_left)
    {
      RCLCPP_INFO(LOGGER, "[left_arm] plan SUCCESS, executing...");
      // Exécution
      left_pc->execute();
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "[left_arm] plan FAILED");
    }

    /* --------------- ÉTAPE 2 : BOTH_ARMS en joint-space (après) --------------- */

    // Récupérer l'état courant
    auto current_state = *moveit_cpp->getCurrentState();
    std::vector<double> q_both;
    current_state.copyJointGroupPositions(jmg_both, q_both);

    if (q_both.size() != 14)
    {
        RCLCPP_ERROR(LOGGER, "[both_arms] Attendu 14 joints, reçu %zu. Vérifie ton SRDF / group 'both_arms'.",
                    q_both.size());
    }
    else
    {
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

        moveit::core::RobotState goal_state = current_state;
        goal_state.setJointGroupPositions(jmg_both, q_both);

        both_pc->setStartStateToCurrentState();
        both_pc->setGoal(goal_state);

        moveit_cpp::PlanningComponent::PlanRequestParameters both_params;
        both_params.planning_pipeline = "ompl";
        both_params.planner_id        = "RRTConnectkConfigDefault";
        both_params.planning_time     = 5.0;
        both_params.planning_attempts = 10;

        auto plan_both = both_pc->plan(both_params);
        if (plan_both)
        {
            RCLCPP_INFO(LOGGER, "[both_arms] joint-space plan SUCCESS, executing...");
            both_pc->execute();
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "[both_arms] joint-space plan FAILED");
        }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(LOGGER, "Fatal error: %s", e.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
