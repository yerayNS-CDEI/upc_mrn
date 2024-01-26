#include "exploration_base.h"

class ExporationFirstFrontier : public ExplorationBase
{
protected:
    double dt_replan_, dist_goal_th_;

public:
    ExporationFirstFrontier();

protected:
    bool replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExporationFirstFrontier::ExporationFirstFrontier() : ExplorationBase("exploration_first_frontier")
{
    // Load parameters from ROS params
    declare_parameter("dt_replan", 2.0); // default 20s
    get_parameter("dt_replan", dt_replan_);
    declare_parameter("dist_goal_th", 0.2); // default 0.2m
    get_parameter("dist_goal_th", dist_goal_th_);
}

geometry_msgs::msg::Pose ExporationFirstFrontier::decideGoal()
{
    // Take first valid frontier
    geometry_msgs::msg::Pose g;
    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        // Check if goal is valid and get path length to the goal
        double path_length;
        g.position = frontiers_msg_.frontiers[i].center_point;
        g.orientation = robot_pose_.orientation;
        if (isValidGoal(g, path_length))
            break;
    }

    return g;
}

bool ExporationFirstFrontier::replan()
{
    // Replan ANYWAY if the robot reached or aborted the goal
    if (robot_status_ != 0)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationFirstFrontier::replan! robot_status_ %d", robot_status_);
        return true;
    }

    // time since last goal was sent
    if (goal_time_ > dt_replan_)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationFirstFrontier::replan! Outdated goal %fs (max %fs)", goal_time_, dt_replan_);
            return true;
    }

    // distance to goal
    if (goal_distance_ < dist_goal_th_)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationFirstFrontier::replan! Close enough to goal %fm (min %fm)", goal_distance_, dist_goal_th_);
        return true;
    }

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExporationFirstFrontier>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  return 0;
}