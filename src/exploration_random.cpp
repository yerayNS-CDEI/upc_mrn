#include "exploration_base.h"

class ExplorationRandom : public ExplorationBase
{
  protected:
    double goal_dist_threshold_;  // replan if the goal is closer than goal_dist_threshold_

  public:
    ExplorationRandom(ros::NodeHandle& nh);

  protected:
    bool                replan() override;
    geometry_msgs::Pose decideGoal() override;
};

ExplorationRandom::ExplorationRandom(ros::NodeHandle& nh) : ExplorationBase(nh)
{
    alg_name_ = "exploration_random";

    // Load parameters from ROS params
    nh_.param<double>("goal_dist_threshold", goal_dist_threshold_, 0.1);

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

geometry_msgs::Pose ExplorationRandom::decideGoal()
{
    // generate a random pose in a circle centered at current position and radius bigger than the map size
    double radius = std::max(2 * map_.info.height * map_.info.resolution, 2 * map_.info.width * map_.info.resolution);

    geometry_msgs::Pose g = generateRandomPose(radius, robot_pose_);

    // generate again until we get a valid goal
    double path_length;
    while (!isValidGoal(g, path_length)) g = generateRandomPose(radius, robot_pose_);

    return g;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_random_node");
    ros::NodeHandle   nh("~");
    ExplorationRandom node_exploration(nh);
    ros::Rate         loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        node_exploration.loop();
    }
    return 0;
}
