#include "exploration_base.h"
#include <fstream>


class ExplorationProject : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    double path_length_;    // length of the computed path to the goal
    
    int case_;           // parameter to decide which case to use (time, dist, time or dist, time and dist)
    double param_dist_;  // distance parameter
    double param_time_;  // time parameter

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationProject(ros::NodeHandle& nh);

  protected:
    bool                replan() override;
    geometry_msgs::Pose decideGoal() override;
};

ExplorationProject::ExplorationProject(ros::NodeHandle& nh) : ExplorationBase(nh)
{
    alg_name_ = "exploration_project";

    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // Set the case to be simulated
    nh_.param<int>("case", case_, 4);

    // Set the maximum distance between goal and frontier in which the robot recomputes a new goal
    nh_.param<double>("param_dist", param_dist_, 0.2);

    // Set the time that the distance conditions must be true in which the robot recomputes a new goal
    nh_.param<double>("param_time", param_time_, 1);

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
}

geometry_msgs::Pose ExplorationProject::decideGoal()
{
    geometry_msgs::Pose g;
    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    double i_closest_;                       // closest frontier id
    double closest_frontier_dist_ = 10e9;    // high value enabling to save first frontier distance


    // Iterative process to identify closest frontier
    for (int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
      // Defining position of the frontier to pass it to the isValidGoal() function
      g.position = frontiers_msg_.frontiers[i].center_free_point;

      // Check if goal is valid and get path length to the goal
      bool valid = isValidGoal(g, path_length_);

      // Whenever goal is valid check if it is closer, saving it in this case
      if(valid && (closest_frontier_dist_ > path_length_)){
        i_closest_ = i;
        closest_frontier_dist_ = path_length_;
      }
    }

    // Defining position and orientation
    g.position = frontiers_msg_.frontiers[i_closest_].center_free_point;
    g.orientation = robot_pose_.orientation;
    // To ease computation the same orientation as the current one has been used,
    // as in the replan we are normally recomputing the goals when we are near them
    // and similar orientation should work well.

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////

    return g;
}

bool ExplorationProject::replan()
{
    ////////////////////////////////////////////////////////////////////
    // TODO 3: replan
    ////////////////////////////////////////////////////////////////////


    // Compute time since last goal was sent
    double dt_last_goal_;
    dt_last_goal_ = (ros::Time::now() - time_target_goal_).toSec();

    // Auxiliar variables to know which situation of the considered is accomplished to replan
    int aux0 = 0;
    int aux1 = 0;
    int aux2 = 0;

    // Computation of the distance between goal and closest frontier
    double dist_goal_frontier_;
    double dist_goal_frontier_cte_ = 10e9;
    for (int i = 0; i < frontiers_msg_.frontiers.size(); i++){
      dist_goal_frontier_ = std::sqrt(std::pow(target_goal_.position.x-frontiers_msg_.frontiers[i].center_free_point.x,2)
                                  +std::pow(target_goal_.position.y-frontiers_msg_.frontiers[i].center_free_point.y,2));
      if (dist_goal_frontier_cte_ > dist_goal_frontier_){
        dist_goal_frontier_cte_ = dist_goal_frontier_;
      }
    }


    // CASE 1: recomputes if more time than than the specified has passed since last computation
    if (case_ == 1){
      // TIME HYPOTHESIS
      if (dt_last_goal_ > param_time_){
        robot_status_ = 1;
        ROS_WARN("///////////////////////// \n");
        ROS_WARN("///////////////////////// \n");
        ROS_WARN("-------  CASE 1  -------\n");
        ROS_WARN("///////////////////////// \n");
        ROS_WARN("///////////////////////// \n");
      }
    }

    // CASE 2: recomputes if the distance between the goal and the closest frontier exceeds param_dist or
    //         if the distance between 2 consecutive goals exceeds param_dist
    if (case_ == 2){
      // DISTANCE HYPOTHESIS
      geometry_msgs::Pose target_goal_2_ = decideGoal();
      if (dist_goal_frontier_cte_ > param_dist_){
        aux0 = 1;
      }
      if (std::sqrt(std::pow(target_goal_.position.x-target_goal_2_.position.x,2)+std::pow(target_goal_.position.y-target_goal_2_.position.y,2)) > param_dist_){
        aux1 = 1;
      }

      if (aux0 > 0 || aux1 > 0){
        robot_status_ = 1;
        ROS_WARN("///////////////////////////////// \n");
        ROS_WARN("///////////////////////////////// \n");
        ROS_WARN("-------  CASE 2  ------->  dist_goal_frontier, dist_goal_goal= [%i %i] \n", aux0, aux1);
        ROS_WARN("///////////////////////////////// \n");
        ROS_WARN("///////////////////////////////// \n");
      }
    }

    // CASE 3: recomputes if (the distance between the goal and the closest frontier exceeds param_dist) OR
    //         if (the distance between 2 consecutive goals exceeds param_dist) OR
    //         if (more time than than the specified has passed since last computation)
    //         AND (if 5.5 seconds has passed since last computation)
    if (case_ == 3){
      // TIME + DISTANCE HYPOTHESIS
      geometry_msgs::Pose target_goal_2_ = decideGoal();   // computation of a new goal to see if the actual one is correct (not on a further frontier)
      if (dist_goal_frontier_cte_ > param_dist_){
        aux0 = 1;
      }
      if (std::sqrt(std::pow(target_goal_.position.x-target_goal_2_.position.x,2)+std::pow(target_goal_.position.y-target_goal_2_.position.y,2)) > param_dist_){
        aux1 = 1;
      }
      if (dt_last_goal_ > param_time_){
        aux2 = 1;
      }

      if (aux0 > 0 || aux1 > 0 || aux2 > 0){
        robot_status_ = 1;
        ROS_WARN("//////////////////////////////////// \n");
        ROS_WARN("//////////////////////////////////// \n");
        ROS_WARN("-------  CASE 3  -------> dist_goal_frontier,dist_goal_goal,time = [%i %i %i] \n", aux0, aux1, aux2);
        //ROS_WARN("%f \n", param_dist_);
        ROS_WARN("//////////////////////////////////// \n");
        ROS_WARN("//////////////////////////////////// \n");
      }

      if (dt_last_goal_ > 5.5){
        robot_status_ = 1;
      }
    }

    // CASE 4: recomputes if (the distance between the goal and the closest frontier exceeds param_dist AND if more time than than the specified has passed since last computation) OR
    //         (if the distance between 2 consecutive goals exceeds param_dist AND if more time than than the specified has passed since last computation) OR
    //         and if (5.5 seconds has passed since last computation)
    if (case_ == 4){
      // TIME + DISTANCE HYPOTHESIS
      geometry_msgs::Pose target_goal_2_ = decideGoal();   // computation of a new goal to see if the actual one is correct (not on a further frontier)
      if ((dist_goal_frontier_cte_ > param_dist_) && (dt_last_goal_ > param_time_)){
        aux0 = 1;
      }
      if ((std::sqrt(std::pow(target_goal_.position.x-target_goal_2_.position.x,2)+std::pow(target_goal_.position.y-target_goal_2_.position.y,2)) > param_dist_) && (dt_last_goal_ > param_time_)){
        aux1 = 1;
      }

      if (aux0 > 0 || aux1 > 0 || aux2 > 0){
        robot_status_ = 1;
        ROS_WARN("/////////////////////////////////////////////////// \n");
        ROS_WARN("/////////////////////////////////////////////////// \n");
        ROS_WARN("-------  CASE 4  -------> dist_goal_frontier,dist_goal_goal = [%i %i] \n", aux0, aux1);
        //ROS_WARN("%f %f \n", param_dist_, param_time_);
        ROS_WARN("/////////////////////////////////////////////////// \n");
        ROS_WARN("/////////////////////////////////////////////////// \n");
      }

      if (dt_last_goal_ > 5.5){
        robot_status_ = 1;
      }
    }

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
    ros::init(argc, argv, "exploration_project_node");
    ros::NodeHandle    nh("~");
    ExplorationProject node_exploration(nh);
    ros::Rate          loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        node_exploration.loop();
    }
    return 0;
}
