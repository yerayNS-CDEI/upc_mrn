#include "exploration_base.h"

class ExplorationBiggestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

    double path_length_;    // length of the computed path to the goal
    double i_biggest_;    // biggest frontier id
    double max_length_frontier_;    // maximum length of the current frontiers
    double dist_goal_replan_;    // threshold distance to recompute goal
    double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationBiggestFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationBiggestFrontier::ExplorationBiggestFrontier() : ExplorationBase("exploration_biggest_frontier")
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

geometry_msgs::msg::Pose ExplorationBiggestFrontier::decideGoal()
{
    geometry_msgs::msg::Pose g;

    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    // // EXAMPLE iterating over detected frontiers
    // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    // {
    //   // Accessing different fields
    //   frontiers_msg_.frontiers[i].size;
    //   frontiers_msg_.frontiers[i].center_point.x;
    //   frontiers_msg_.frontiers[i].center_point.y;
    //   frontiers_msg_.frontiers[i].center_point.z;
    // }

    // // EXAMPLE filling Pose message
    // // The goal position can be filled with the center_point of the "best" frontier
    // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
    //
    // // The orientation has to be filled as well.
    // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
    // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

    // // EXAMPLE check if a goal is valid and get path length to the goal
    // double path_length;
    // bool valid = isValidGoal(g, path_length)

    max_length_frontier_ = 0;    // low value enabling to save first frontier length

    // Iterative process to identify biggest frontier
    for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
      // Defining position of the frontier to pass it to the isValidGoal() function
      g.position = frontiers_msg_.frontiers[i].center_point;

      // Check if goal is valid and get path length to the goal
      bool valid = isValidGoal(g, path_length_);

      // Whenever goal is valid check if it is bigger, saving it in this case
      if(valid && (max_length_frontier_ < frontiers_msg_.frontiers[i].size)){
        i_biggest_ = i;
        max_length_frontier_ = frontiers_msg_.frontiers[i].size;
      }
    }

    // Defining position and orientation
    g.position = frontiers_msg_.frontiers[i_biggest_].center_point;
    g.orientation = robot_pose_.orientation;
    // To ease computation the same orientation as the current one has been used,
    // as in the replan we are normally recomputing the goals when we are near them
    // and similar orientation should work well.

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////

    return g;
}

bool ExplorationBiggestFrontier::replan()
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
  auto node = std::make_shared<ExplorationBiggestFrontier>();
  exec.add_node(node);
  exec.spin();

  return 0;
}
