////////////////////////////////
/// ORIGINAL
////////////////////////////////

#include "exploration_base.h"

class ExplorationClosestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

    double path_length_;    // length of the computed path to the goal
    int i_closest_;    // closest frontier id
    double closest_frontier_dist_;    // minimum distance to the current frontiers
    double dist_goal_;    // cartesian distance to goal
    double dt_last_goal_;    // time elapsed since last goal sent
    double dist_goal_replan_;    // threshold distance to recompute goal
    double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationClosestFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
{
    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
    // // If the parameters is not defined, use the default value 20:
    // get_parameter_or("time_max_goal", time_max_goal_, 20);

    // Get the maximum distance to the goal in which the robot recomputes a new goal
    get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

    // Get the maximum amount of time elapsed between goals computation
    get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
}

geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
{
    geometry_msgs::msg::Pose g;

    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    closest_frontier_dist_ = 10e9;
    i_closest_ = -1;

    RCLCPP_INFO(this->get_logger(), "Frontiers recibidas: %zu", frontiers_msg_.frontiers.size());

    for (int i = 0; i < static_cast<int>(frontiers_msg_.frontiers.size()); i++)
    {
      g.position = frontiers_msg_.frontiers[i].center_point;
      double path_len = -1.0;
      bool valid = isValidGoal(g, path_len);

      // Log por frontera
      const double dx = g.position.x - robot_pose_.position.x;
      const double dy = g.position.y - robot_pose_.position.y;
      const double eucl = std::hypot(dx, dy);
      RCLCPP_INFO(this->get_logger(),
          "[F%d] pos=(%.2f, %.2f) eucl=%.2f valid=%s path_len=%.2f",
          i, g.position.x, g.position.y, eucl, (valid?"true":"false"), path_len);

      if (valid && (closest_frontier_dist_ > path_len)){
        i_closest_ = i;
        closest_frontier_dist_ = path_len;
        RCLCPP_INFO(this->get_logger(),
            " -> candidato por PATH: F%d con path_len=%.2f", i_closest_, closest_frontier_dist_);
      }
    }

    if (i_closest_ < 0) {
      RCLCPP_WARN(this->get_logger(),
          "Ninguna frontera con PATH válido. decideGoal() devolverá la primera invalida y decideGoalBase() lanzará goal aleatoria.");
    } else {
      RCLCPP_INFO(this->get_logger(),
          "Frontier elegida por PATH: F%d (path_len=%.2f).",
          i_closest_, closest_frontier_dist_);
    }

    if (i_closest_ >= 0) {
      g.position = frontiers_msg_.frontiers[i_closest_].center_point;
      g.orientation = robot_pose_.orientation;
    } else {
      // No hubo path válido: deja g tal cual (el último), o si prefieres:
      // g = robot_pose_;  // y decideGoalBase() hará el fallback aleatorio
    }

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////

    return g;
}

bool ExplorationClosestFrontier::replan()
{
    // REMEMBER:
    // goal_time_ has the time since last goal was sent (seconds)
    // goal_distance_ has remaining distance to reach the last goal (meters)

    ////////////////////////////////////////////////////////////////////
    // TODO 3: replan
    ////////////////////////////////////////////////////////////////////

    // Recompute a goal if the robot is near the previous one
    // or more time than the specified has passed since last computation
    if (goal_distance_ < dist_goal_replan_ || goal_time_ > dt_last_goal_replan_){
      robot_status_ = 1;
    }
    
    ////////////////////////////////////////////////////////////////////
    // TODO 3 END
    ////////////////////////////////////////////////////////////////////

    // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
    if (robot_status_ != 0) return true;

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExplorationClosestFrontier>();
  exec.add_node(node);
  exec.spin();

  return 0;
}










// #include "exploration_base.h"

// #include <limits>
// #include <cmath>
// #include <algorithm>
// #include <vector>

// class ExplorationClosestFrontier : public ExplorationBase
// {
//   protected:
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a: if you need your own attributes (variables) and/or
//     //          methods (functions), define them HERE
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE ATTRIBUTE:
//     // double time_max_goal_; // max time to reach a goal before aborting

//     double path_length_;    // length of the computed path to the goal
//     double i_closest_;    // closest frontier id
//     double closest_frontier_dist_;    // minimum distance to the current frontiers
//     double dist_goal_;    // cartesian distance to goal
//     double dt_last_goal_;    // time elapsed since last goal sent
//     double dist_goal_replan_;    // threshold distance to recompute goal
//     double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

//     double startup_delay_sec_;
//     int    validate_top_k_;
//     rclcpp::Time node_start_time_;

//     int    frontier_free_search_rings_;
//     int    frontier_free_steps_;
//     double frontier_min_clearance_m_;

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a END
//     //////////////////////////////////////////////////////////////////////

//   public:
//     ExplorationClosestFrontier();

//   protected:
//     bool                replan() override;
//     geometry_msgs::msg::Pose decideGoal() override;
// };

// ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
// {
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b: You can set the value of attributes using ros param
//     //          for changing the value without need of recompiling.
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
//     // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
//     // // If the parameters is not defined, use the default value 20:
//     // get_parameter_or("time_max_goal", time_max_goal_, 20);

//     // Get the maximum distance to the goal in which the robot recomputes a new goal
//     get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

//     // Get the maximum amount of time elapsed between goals computation
//     get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);

//     get_parameter_or("startup_delay_sec", startup_delay_sec_, 3.0);
//     get_parameter_or("validate_top_k",     validate_top_k_,     12);
//     node_start_time_ = now();

//     get_parameter_or("frontier_free_search_rings",  frontier_free_search_rings_,  5);  // ~5 celdas (~0.25 m si res=0.05)
//     get_parameter_or("frontier_free_steps",        frontier_free_steps_,        24);  // 24 direcciones
//     get_parameter_or("frontier_min_clearance_m",   frontier_min_clearance_m_,  0.10); // 10 cm de “aire” alrededor

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b END
//     //////////////////////////////////////////////////////////////////////
// }

// geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
// {
//     geometry_msgs::msg::Pose g;

//     auto isFreeWithClearance = [&](const geometry_msgs::msg::Point& p)->bool {
//       if (!isFree(p)) return false;
//       // comprueba 8-vecindad a una distancia mínima
//       const double res = map_.info.resolution;
//       const double r   = std::max(res, frontier_min_clearance_m_);
//       for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
//         if (dx==0 && dy==0) continue;
//         geometry_msgs::msg::Point q{p.x + dx*r, p.y + dy*r, 0.0};
//         if (!isFree(q)) return false;
//       }
//       return true;
//     };

//     auto nearestFreeAround = [&](const geometry_msgs::msg::Point& c,
//                                 geometry_msgs::msg::Point& best)->bool {
//       const double res = map_.info.resolution;
//       for (int ring = 0; ring <= frontier_free_search_rings_; ++ring) {
//         double R = ring * res; // 0, 1*res, 2*res, ...
//         int K    = std::max(8, frontier_free_steps_);
//         for (int k = 0; k < K; ++k) {
//           double th = (2.0 * M_PI * k) / K;
//           geometry_msgs::msg::Point p{ c.x + R*std::cos(th), c.y + R*std::sin(th), 0.0 };
//           if (isFreeWithClearance(p)) { best = p; return true; }
//         }
//       }
//       return false;
//     };

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2: decide goal
//     ////////////////////////////////////////////////////////////////////

//     // // EXAMPLE iterating over detected frontiers
//     // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
//     // {
//     //   // Accessing different fields
//     //   frontiers_msg_.frontiers[i].size;
//     //   frontiers_msg_.frontiers[i].center_point.x;
//     //   frontiers_msg_.frontiers[i].center_point.y;
//     //   frontiers_msg_.frontiers[i].center_point.z;
//     // }

//     // // EXAMPLE filling Pose message
//     // // The goal position can be filled with the center_point of the "best" frontier
//     // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
//     //
//     // // The orientation has to be filled as well.
//     // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
//     // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

//     // // EXAMPLE check if a goal is valid and get path length to the goal
//     // double path_length;
//     // bool valid = isValidGoal(g, path_length)

//     // 0) Sin fronteras -> usa pose del robot (no fuerces random)
//     if (frontiers_msg_.frontiers.empty()) {
//       RCLCPP_WARN(this->get_logger(), "[exploration] No frontiers available.");
//       return robot_pose_;
//     }

//     // 1) Ordena por distancia euclídea y valida sólo las K más cercanas
//     std::vector<std::pair<double,int>> cand;
//     cand.reserve(frontiers_msg_.frontiers.size());
//     for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//       const auto& p = frontiers_msg_.frontiers[i].center_point;
//       double dx = p.x - robot_pose_.position.x;
//       double dy = p.y - robot_pose_.position.y;
//       cand.emplace_back(dx*dx + dy*dy, i);
//     }
//     std::sort(cand.begin(), cand.end(), [](auto& a, auto& b){ return a.first < b.first; });

//     closest_frontier_dist_ = std::numeric_limits<double>::infinity();
//     i_closest_ = -1;
//     geometry_msgs::msg::Pose g_best;

//     const int K = std::max(1, validate_top_k_);
//     for (int j = 0; j < (int)cand.size() && j < K; ++j) {
//       int i = cand[j].second;
      
//       // 0) buscar punto libre cercano al centro (descarta columnas)
//       geometry_msgs::msg::Point p_free;
//       if (!nearestFreeAround(frontiers_msg_.frontiers[i].center_point, p_free)) {
//         // columna o encerrada -> ignora esta frontera
//         continue;
//       }

//       // 1) candidato sobre celda libre
//       g.position = p_free;
//       g.position.z = 0.0;

//       // orientar mirando al objetivo
//       double yaw = std::atan2(g.position.y - robot_pose_.position.y,
//                               g.position.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       g.orientation = tf2::toMsg(q);

//       // 2) valida y quédate con el mejor por longitud de camino
//       if (isValidGoal(g, path_length_) && path_length_ < closest_frontier_dist_) {
//         closest_frontier_dist_ = path_length_;
//         i_closest_ = i;
//         g_best = g;
//       }
//     }

//     // 2) Si ninguna pasó el validador, usa la más cercana euclídea sin invalidar índices
//     if (i_closest_ < 0) {
//       int i = cand.front().second;
//       geometry_msgs::msg::Point p;
//       if (!nearestFreeAround(frontiers_msg_.frontiers[i].center_point, p)) {
//         // ni siquiera hay libre cerca: devuelve robot_pose_ y deja que la base haga random
//         RCLCPP_WARN(this->get_logger(), "[exploration] Frontier near (%0.2f,%0.2f) has no free neighbors; skipping",
//                     frontiers_msg_.frontiers[i].center_point.x,
//                     frontiers_msg_.frontiers[i].center_point.y);
//         return robot_pose_;
//       }
//       g_best.position = p;
//       g_best.position.z = 0.0;
//       double yaw = std::atan2(p.y - robot_pose_.position.y,
//                               p.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       g_best.orientation = tf2::toMsg(q);
//     }

//     // Revalidación final + pequeño empujón hacia el robot si aún no pasa
//     double _L;
//     if (!isValidGoal(g_best, _L)) {
//       auto p = g_best.position;
//       const auto& rp = robot_pose_.position;
//       double dx = rp.x - p.x, dy = rp.y - p.y;
//       double n = std::hypot(dx, dy);
//       if (n > 1e-6) {
//         p.x += 0.20 * dx / n;
//         p.y += 0.20 * dy / n;
//         p.z  = 0.0;
//         g_best.position = p;
//         double yaw = std::atan2(p.y - rp.y, p.x - rp.x);
//         tf2::Quaternion q; q.setRPY(0,0,yaw);
//         g_best.orientation = tf2::toMsg(q);
//         (void) isValidGoal(g_best, _L); // segundo intento
//       }
//     }
//     ////////////////////////////////////////////////////////////////////
//     // TODO 2 END
//     ////////////////////////////////////////////////////////////////////

//     return g_best;
// }

// bool ExplorationClosestFrontier::replan()
// {
//     // REMEMBER:
//     // goal_time_ has the time since last goal was sent (seconds)
//     // goal_distance_ has remaining distance to reach the last goal (meters)

//     ////////////////////////////////////////////////////////////////////
//     // TODO 3: replan
//     ////////////////////////////////////////////////////////////////////

//     if ((now() - node_start_time_).seconds() < startup_delay_sec_) {
//       return false;
//     }

//     // Recompute a goal if the robot is near the previous one
//     // or more time than the specified has passed since last computation
//     if (goal_distance_ < dist_goal_replan_ || goal_time_ > dt_last_goal_replan_){
//       robot_status_ = 1;
//     }
    
//     ////////////////////////////////////////////////////////////////////
//     // TODO 3 END
//     ////////////////////////////////////////////////////////////////////

//     // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
//     if (robot_status_ != 0) return true;

//     return false;
// }

// ////// MAIN ////////////////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::executors::MultiThreadedExecutor exec;
//   auto node = std::make_shared<ExplorationClosestFrontier>();
//   exec.add_node(node);
//   exec.spin();

//   return 0;
// }















// #include "exploration_base.h"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <limits>
// #include <cmath>

// class ExplorationClosestFrontier : public ExplorationBase
// {
//   protected:
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a: if you need your own attributes (variables) and/or
//     //          methods (functions), define them HERE
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE ATTRIBUTE:
//     // double time_max_goal_; // max time to reach a goal before aborting

//     double path_length_;    // length of the computed path to the goal
//     int i_closest_;    // closest frontier id
//     double closest_frontier_dist_;    // minimum distance to the current frontiers
//     double dist_goal_;    // cartesian distance to goal
//     double dt_last_goal_;    // time elapsed since last goal sent
//     double dist_goal_replan_;    // threshold distance to recompute goal
//     double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

//     // Snapping del goal cerca del centro de la frontera
//     double goal_snap_max_radius_;    // m (p. ej., 0.6)
//     double goal_snap_step_;          // m (p. ej., 0.1)
//     int    goal_snap_num_angles_;    // divisiones angulares (p. ej., 16)
//     double goal_nudge_towards_robot_; // m, empuja el goal ligeramente hacia el robot (p. ej., 0.15)

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a END
//     //////////////////////////////////////////////////////////////////////

//   public:
//     ExplorationClosestFrontier();

//   protected:
//     bool                replan() override;
//     geometry_msgs::msg::Pose decideGoal() override;
// };

// ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
// {
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b: You can set the value of attributes using ros param
//     //          for changing the value without need of recompiling.
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
//     // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
//     // // If the parameters is not defined, use the default value 20:
//     // get_parameter_or("time_max_goal", time_max_goal_, 20);

//     // Get the maximum distance to the goal in which the robot recomputes a new goal
//     get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

//     // Get the maximum amount of time elapsed between goals computation
//     get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);

//     get_parameter_or("goal_snap_max_radius",     goal_snap_max_radius_,     0.8);
//     get_parameter_or("goal_snap_step",           goal_snap_step_,           0.15);
//     get_parameter_or("goal_snap_num_angles",     goal_snap_num_angles_,     24);
//     get_parameter_or("goal_nudge_towards_robot", goal_nudge_towards_robot_, 0.25);

//     this->set_parameter(rclcpp::Parameter("robot_base_frame", "turret_link"));
//     this->set_parameter(rclcpp::Parameter("robot_frame", "turret_link"));
//     this->set_parameter(rclcpp::Parameter("global_frame", "map"));
//     this->set_parameter(rclcpp::Parameter("odom_frame", "odom"));

//     this->set_parameter(rclcpp::Parameter("tf_timeout", 0.2));

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b END
//     //////////////////////////////////////////////////////////////////////
// }

// geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
// {
//     geometry_msgs::msg::Pose g;

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2: decide goal
//     ////////////////////////////////////////////////////////////////////

//     // // EXAMPLE iterating over detected frontiers
//     // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
//     // {
//     //   // Accessing different fields
//     //   frontiers_msg_.frontiers[i].size;
//     //   frontiers_msg_.frontiers[i].center_point.x;
//     //   frontiers_msg_.frontiers[i].center_point.y;
//     //   frontiers_msg_.frontiers[i].center_point.z;
//     // }

//     // // EXAMPLE filling Pose message
//     // // The goal position can be filled with the center_point of the "best" frontier
//     // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
//     //
//     // // The orientation has to be filled as well.
//     // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
//     // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

//     // // EXAMPLE check if a goal is valid and get path length to the goal
//     // double path_length;
//     // bool valid = isValidGoal(g, path_length)

//     if (frontiers_msg_.frontiers.empty()) {
//         RCLCPP_WARN(this->get_logger(), "[exploration] No frontiers available.");
//         return robot_pose_;
//     }

//     auto make_pose = [&](const geometry_msgs::msg::Point& p)->geometry_msgs::msg::Pose {
//       geometry_msgs::msg::Pose out;
//       out.position = p;
//       // orientar hacia el goal (mejora la validez en algunos validadores)
//       double yaw = std::atan2(p.y - robot_pose_.position.y, p.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       out.orientation = tf2::toMsg(q);
//       return out;
//     };

//     auto snap_goal = [&](const geometry_msgs::msg::Point& center,
//                         geometry_msgs::msg::Pose& snapped,
//                         double& best_len)->bool {
//       // nudge hacia el robot
//       geometry_msgs::msg::Point seed = center;
//       {
//         double vx = center.x - robot_pose_.position.x;
//         double vy = center.y - robot_pose_.position.y;
//         double n  = std::hypot(vx, vy);
//         if (n > 1e-6) {
//           seed.x -= goal_nudge_towards_robot_ * (vx / n);
//           seed.y -= goal_nudge_towards_robot_ * (vy / n);
//         }
//       }
//       geometry_msgs::msg::Pose cand = make_pose(seed);
//       if (isValidGoal(cand, best_len)) { snapped = cand; return true; }

//       // anillos concéntricos
//       const int   K   = std::max(4, goal_snap_num_angles_);
//       const double Rm = std::max(0.0, goal_snap_max_radius_);
//       const double dR = std::max(0.02, goal_snap_step_);

//       bool found = false;
//       double best = std::numeric_limits<double>::infinity();
//       geometry_msgs::msg::Pose best_pose;

//       for (double r = dR; r <= Rm + 1e-6; r += dR) {
//         for (int k = 0; k < K; ++k) {
//           double th = (2.0 * M_PI * k) / K;
//           geometry_msgs::msg::Point p;
//           p.x = seed.x + r * std::cos(th);
//           p.y = seed.y + r * std::sin(th);
//           p.z = 0.0;
//           geometry_msgs::msg::Pose cand_k = make_pose(p);
//           double len_k;
//           if (isValidGoal(cand_k, len_k) && len_k < best) {
//             best = len_k; best_pose = cand_k; found = true;
//           }
//         }
//         if (found && best < 0.5 * r) break; // salir temprano si ya tenemos uno bueno
//       }
//       if (found) { snapped = best_pose; best_len = best; }
//       return found;
//     };

//     closest_frontier_dist_ = 1e12;
//     i_closest_ = -1;
//     geometry_msgs::msg::Pose g_best;

//     for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//       const auto& f = frontiers_msg_.frontiers[i];
//       geometry_msgs::msg::Pose g_try;
//       double len_try;
//       bool ok = snap_goal(f.center_point, g_try, len_try);
//       if (ok && len_try < closest_frontier_dist_) {
//         closest_frontier_dist_ = len_try;
//         i_closest_ = i;
//         g_best = g_try;
//       }
//     }
//     RCLCPP_INFO(this->get_logger(),
//             "[exploration] Picked frontier id=%d, path=%.2f m, goal=(%.2f, %.2f)",
//             i_closest_, closest_frontier_dist_, g_best.position.x, g_best.position.y);


//     // Fallback: euclídea si ninguna pasó el validador
//     if (i_closest_ < 0) {
//       double best_d2 = std::numeric_limits<double>::infinity();
//       int best_i = -1;
//       for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//         const auto& p = frontiers_msg_.frontiers[i].center_point;
//         double dx = p.x - robot_pose_.position.x;
//         double dy = p.y - robot_pose_.position.y;
//         double d2 = dx*dx + dy*dy;
//         if (d2 < best_d2) { best_d2 = d2; best_i = i; }
//       }
//       if (best_i >= 0) {
//         i_closest_ = best_i;
//         g_best = make_pose(frontiers_msg_.frontiers[best_i].center_point);
//         RCLCPP_WARN(this->get_logger(),
//           "[exploration] All snapped goals invalid; falling back to Euclidean id=%d", i_closest_);
//       } else {
//         // último salvavidas: NO devuelvas (0,0,0), usa robot_pose_
//         RCLCPP_WARN(this->get_logger(), "[exploration] No usable frontier; returning robot pose.");
//         return robot_pose_;
//       }
//     }

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2 END
//     ////////////////////////////////////////////////////////////////////

//     double _relen;
//     if (!isValidGoal(g_best, _relen)) {
//       // nudge 0.20 m hacia el robot para sacar el goal del borde de obstáculo/desconocido
//       geometry_msgs::msg::Point p = g_best.position;
//       double dx = robot_pose_.position.x - p.x, dy = robot_pose_.position.y - p.y;
//       double n = std::hypot(dx, dy);
//       if (n > 1e-6) { p.x += 0.20 * dx / n; p.y += 0.20 * dy / n; }
//       geometry_msgs::msg::Pose g_retry = make_pose(p);
//       if (isValidGoal(g_retry, _relen)) { g_best = g_retry; }
//     }
//     return g_best;
// }

// bool ExplorationClosestFrontier::replan()
// {
//     // REMEMBER:
//     // goal_time_ has the time since last goal was sent (seconds)
//     // goal_distance_ has remaining distance to reach the last goal (meters)

//     ////////////////////////////////////////////////////////////////////
//     // TODO 3: replan
//     ////////////////////////////////////////////////////////////////////

//     // Recompute a goal if the robot is near the previous one
//     // or more time than the specified has passed since last computation
//     if (goal_distance_ < dist_goal_replan_ || goal_time_ > dt_last_goal_replan_){
//       robot_status_ = 1;
//     }
    
//     ////////////////////////////////////////////////////////////////////
//     // TODO 3 END
//     ////////////////////////////////////////////////////////////////////

//     // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
//     if (robot_status_ != 0) return true;

//     return false;
// }

// ////// MAIN ////////////////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::executors::MultiThreadedExecutor exec;
//   auto node = std::make_shared<ExplorationClosestFrontier>();
//   exec.add_node(node);
//   exec.spin();

//   return 0;
// }
