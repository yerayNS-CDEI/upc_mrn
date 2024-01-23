#include "exploration_base.h"

class ExporationFirstFrontier : public ExplorationBase
{
  protected:
    double dt_replan_, dist_goal_th_;

  public:
    ExporationFirstFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::Pose decideGoal() override;
};

ExporationFirstFrontier::ExporationFirstFrontier() : ExplorationBase("ExporationFirstFrontier")
{
    nh_.param<double>("dt_replan", dt_replan_, 20);
    nh_.param<double>("dist_goal_th", dist_goal_th_, 0.2);
}

geometry_msgs::Pose ExporationFirstFrontier::decideGoal()
{
    // Take first valid frontier
    geometry_msgs::Pose g;
    for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        // Check if goal is valid and get path length to the goal
        double path_length;
        g.position    = frontiers_msg_.frontiers[i].center_free_point;
        g.orientation = robot_pose_.orientation;
        if (isValidGoal(g, path_length)) break;
    }

    return g;
}

bool ExporationFirstFrontier::replan()
{
    // Replan ANYWAY if the robot reached or aborted the goal
    if (robot_status_ != 0) return true;

    // time since last goal was sent
    if (goal_time_ > dt_replan_) return true;

    // distance to goal
    if (goal_distance_ < dist_goal_th_) return true;

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_first_frontier_node");
    ros::NodeHandle         nh("~");
    ExporationFirstFrontier node_exploration(nh);
    ros::Rate               loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        node_exploration.loop();
    }
    return 0;
}
