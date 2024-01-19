#include "exploration_base.h"

namespace upc_mrn
{

class ExplorationRandom : public ExplorationBase
{
  protected:
    double goal_dist_threshold_;  // replan if the goal is closer than goal_dist_threshold_

  public:
    ExplorationRandom();

  protected:
    bool                replan() override;
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
    // Compute distance to goal
    double dist_goal = std::sqrt(std::pow(target_goal_.position.x - robot_pose_.position.x, 2) +
                                 std::pow(target_goal_.position.y - robot_pose_.position.y, 2));

    // Replan if the goal is closer than 'goal_dist_threshold_' m
    if (dist_goal < goal_dist_threshold_) return true;

    // Replan ANYWAY if the robot (reached or) aborted the goal
    if (robot_status_ != 0) return true;

    return false;
}

geometry_msgs::msg::Pose ExplorationRandom::decideGoal()
{
    // generate a random pose in a circle centered at current position and radius bigger than the map size
    double radius = std::max(2 * map_.info.height * map_.info.resolution, 2 * map_.info.width * map_.info.resolution);

    geometry_msgs::msg::Pose g = generateRandomPose(radius, robot_pose_);

    // generate again until we get a valid goal
    double path_length;
    while (!isValidGoal(g, path_length)) g = generateRandomPose(radius, robot_pose_);

    return g;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(upc_mrn::ExplorationRandom)

////// MAIN ////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<upc_mrn::ExplorationRandom>());
  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "exploration_random_node");
//     ros::NodeHandle   nh("~");
//     ExplorationRandom node_exploration(nh);
//     ros::Rate         loop_rate(10);

//     while (ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();

//         node_exploration.loop();
//     }
//     return 0;
// }

