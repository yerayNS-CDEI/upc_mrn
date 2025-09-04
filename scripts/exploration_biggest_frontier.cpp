#include "exploration_base.h"

class ExplorationBiggestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or 
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationBiggestFrontier(ros::NodeHandle& nh);

  protected:
    bool                replan() override;
    geometry_msgs::Pose decideGoal() override;
};

ExplorationBiggestFrontier::ExplorationBiggestFrontier(ros::NodeHandle& nh) : ExplorationBase(nh)
{
    alg_name_ = "exploration_biggest_frontier";

    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param 
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // Set the attribute 'time_max_goal_' the value of param "time_max_goal" if defined, 
    // otherwise, it is set to 20.
    // nh_.param<double>("time_max_goal", time_max_goal_, 20); 

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
}

geometry_msgs::Pose ExplorationBiggestFrontier::decideGoal()
{
    // EXAMPLE iterating over detected frontiers
    // for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    // {
    //   // EXAMPLE Accessing different fields
    //   frontiers_msg_.frontiers[i].id;
    //   frontiers_msg_.frontiers[i].size;
    //   frontiers_msg_.frontiers[i].center_free_point.x;
    //   frontiers_msg_.frontiers[i].center_free_point.y;
    //   frontiers_msg_.frontiers[i].center_free_point.z;
    //
    //   // EXAMPLE check if goal is valid and get path length to the goal
    //   double path_length;
    //   bool valid = isValidGoal(frontiers_msg_.frontiers[i].center_free_point, path_length)
    //}

    geometry_msgs::Pose g;
    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    // EXAMPLE
    // The goal position can be filled with the center_free_point of the "best" frontier
    // g.position = frontiers_msg_.frontiers[i_best].center_free_point;

    // EXAMPLE
    // The orientation has to be filled as well.
    // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
    // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero theta

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////

    return g;
}

bool ExplorationBiggestFrontier::replan()
{
    // EXAMPLES:
    // Compute time since last goal was sent
    // double dt_last_goal = (ros::Time::now() - time_target_goal_).toSec();
    // Compute distance to goal
    // double dist_goal = std::sqrt(std::pow(target_goal_.position.x-robot_pose_.position.x,2)
    //                             +std::pow(target_goal_.position.y-robot_pose_.position.y,2));

    ////////////////////////////////////////////////////////////////////
    // TODO 3: replan
    ////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////
    // TODO 3 END
    ////////////////////////////////////////////////////////////////////

    // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
    if (robot_status_ != 0) return true;

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_biggest_frontier_node");
    ros::NodeHandle            nh("~");
    ExplorationBiggestFrontier node_exploration(nh);
    ros::Rate                  loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        node_exploration.loop();
    }
    return 0;
}
