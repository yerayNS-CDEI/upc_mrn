#include "exploration_base.h"

class ExplorationBiggestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    double path_length_;    // length of the computed path to the goal
    double i_biggest_;    // biggest frontier id
    double max_length_frontier_;    // maximum length of the current frontiers
    double dist_goal_;    // distance to goal
    double dt_last_goal_;    // time elapsed since last goal was sent
    double dist_goal_replan_;    // threshold distance to recompute goal
    double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

    // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

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

    // Set the maximum distance to the goal in which the robot recomputes a new goal
    nh_.param<double>("dist_goal_replan", dist_goal_replan_, 1);

    // Set the maximum amount of time elapsed between goals computation
    nh_.param<double>("dt_last_goal_replan", dt_last_goal_replan_, 4);

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////

    // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // Set the attribute 'time_max_goal_' the value of param "time_max_goal" if defined,
    // otherwise, it is set to 20.
    // nh_.param<double>("time_max_goal", time_max_goal_, 20);
}

geometry_msgs::Pose ExplorationBiggestFrontier::decideGoal()
{
    geometry_msgs::Pose g;
    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    max_length_frontier_ = 0;    // low value enabling to save first frontier length

    // Iterative process to identify biggest frontier
    for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
      // Defining position of the frontier to pass it to the isValidGoal() function
      g.position = frontiers_msg_.frontiers[i].center_free_point;

      // Check if goal is valid and get path length to the goal
      bool valid = isValidGoal(g, path_length_);

      // Whenever goal is valid check if it is bigger, saving it in this case
      if(valid && (max_length_frontier_ < frontiers_msg_.frontiers[i].size)){
        i_biggest_ = i;
        max_length_frontier_ = frontiers_msg_.frontiers[i].size;
      }
    }

    // Defining position and orientation
    g.position = frontiers_msg_.frontiers[i_biggest_].center_free_point;
    g.orientation = robot_pose_.orientation;
    // To ease computation the same orientation as the current one has been used,
    // as in the replan we are normally recomputing the goals when we are near them
    // and similar orientation should work well.

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////

    // EXAMPLE iterating over detected frontiers
    // for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    // {
    //   // Accessing different fields
    //   frontiers_msg_.frontiers[i].id;
    //   frontiers_msg_.frontiers[i].size;
    //   frontiers_msg_.frontiers[i].center_free_point.x;
    //   frontiers_msg_.frontiers[i].center_free_point.y;
    //   frontiers_msg_.frontiers[i].center_free_point.z;
    // }

    // EXAMPLE
    // The goal position can be filled with the center_free_point of the "best" frontier
    // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
    //
    // The orientation has to be filled as well.
    // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
    // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero theta

    // // EXAMPLE check if a goal is valid and get path length to the goal
    // double path_length;
    // bool valid = isValidGoal(g, path_length)

    return g;
}

bool ExplorationBiggestFrontier::replan()
{

    ////////////////////////////////////////////////////////////////////
    // TODO 3: replan
    ////////////////////////////////////////////////////////////////////

    // Compute time since last goal was sent
    dt_last_goal_ = (ros::Time::now() - time_target_goal_).toSec();

    // Compute distance to goal
    dist_goal_ = std::sqrt(std::pow(target_goal_.position.x-robot_pose_.position.x,2)
                                +std::pow(target_goal_.position.y-robot_pose_.position.y,2));

    // Recompute a goal if the robot is near the previous one
    // or more time than the specified has passed since last computation
    if (dist_goal_ < dist_goal_replan_ || dt_last_goal_ > dt_last_goal_replan_){
      robot_status_ = 1;
    }

    ////////////////////////////////////////////////////////////////////
    // TODO 3 END
    ////////////////////////////////////////////////////////////////////

    // EXAMPLES:
    // Compute time since last goal was sent
    // double dt_last_goal_ = (ros::Time::now() - time_target_goal_).toSec();
    // Compute distance to goal
    // double dist_goal_ = std::sqrt(std::pow(target_goal_.position.x-robot_pose_.position.x,2)
    //                             +std::pow(target_goal_.position.y-robot_pose_.position.y,2));

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
