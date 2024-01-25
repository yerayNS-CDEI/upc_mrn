#include "exploration_base.h"

class ExplorationRandom : public ExplorationBase
{
protected:
  double goal_dist_threshold_; // replan if the goal is closer than goal_dist_threshold_

public:
  ExplorationRandom();

protected:
  bool replan() override;
  geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationRandom::ExplorationRandom() : ExplorationBase("exploration_random")
{
  // Load parameters from ROS params
  declare_parameter("goal_dist_threshold", 0.1); // default 0.1m
  get_parameter("goal_dist_threshold", goal_dist_threshold_);

  // Start exploration
  exploration_started_ = true;
}

bool ExplorationRandom::replan()
{
  RCLCPP_INFO(this->get_logger(), "ExplorationRandom::replan");

  // Replan if the goal is closer than 'goal_dist_threshold_' m
  if (goal_distance_ < goal_dist_threshold_)
    return true;

  // Replan ANYWAY if the robot (reached or) aborted the goal
  if (robot_status_ != 0)
    return true;

  return false;
}

geometry_msgs::msg::Pose ExplorationRandom::decideGoal()
{
  RCLCPP_INFO(this->get_logger(), "ExplorationRandom::decideGoal");

  // generate a random pose in a circle centered at current position and radius bigger than the map size
  double radius = std::max(2 * map_.info.height * map_.info.resolution, 2 * map_.info.width * map_.info.resolution);

  geometry_msgs::msg::Pose g = generateRandomPose(radius, robot_pose_);

  // generate again until we get a valid goal
  double path_length;
  while (not isValidGoal(g, path_length))
    g = generateRandomPose(radius, robot_pose_);

  RCLCPP_INFO(this->get_logger(), "goal");

  return g;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExplorationRandom>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  return 0;
}
