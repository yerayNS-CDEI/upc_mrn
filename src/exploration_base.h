#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "upc_mrn/msg/frontiers.hpp"
#include "nav_msgs/srv/get_plan.hpp"

// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>

// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>

class ExplorationBase : public Node
{
protected:
    // ------------------ ATTRIBUTES ------------------
    std::string algorithm_variant_;

    rclcpp::Subscription<upc_mrn::msg::Frontiers>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_new_pub_;

    // Planner actions TODO
    // ros::ServiceClient get_plan_client_;
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_move_base_;

    tf::TransformListener listener_;

    int robot_status_;                     // 0: moving, 1: goal reached, 2: failed to reach goal
    geometry_msgs::msg::Pose target_goal_; // last goal sent
    geometry_msgs::msg::Pose robot_pose_;  // current robot pose

    nav_msgs::msg::OccupancyGrid map_;
    upc_mrn::msg::Frontiers frontiers_msg_;

    // STATS
    int num_cells_;                  // amount of map cells (height*width)
    int cells_explored_;             // explored cells (free or obstacle)
    int num_goals_sent_;             // total goals sent
    int num_goals_ko_;               // total goals cancelled
    double distance_traveled_;       // total absolute distance traveled during exploration (m)
    double angle_traveled_;          // total absolute angle turned during exploration (rad)
    rclcpp::Time begin_exploration_; // used to compute the total exploration time
    rclcpp::Time time_target_goal_;  // time when last goal was sent
    bool exploration_ended_;         // flag to finish exploration
    bool exploration_started_;       // flag to start exploration
    bool got_map_;                   // flag to ack the map reception
    std::string results_file_;       // name of the results file (csv)
    std::string scene_;              // name of the scene

    // ------------------ METHODS ------------------
public:
    ExplorationBase(const std::string &_name);
    ~ExplorationBase();
    void loop();

protected:
    // CALLBACKS (inputs)
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void frontiersCallback(const upc_mrn::Frontiers::ConstPtr &msg);

    // EXPLORATION ALGORITHM MAIN FUNCTIONS
    // Functions to be implemented in derived class
    virtual bool replan() = 0;
    virtual geometry_msgs::Pose decideGoal() = 0;
    // Base class functions
    virtual geometry_msgs::Pose decideGoalBase();
    void finish();

    // AUXILIARY
    geometry_msgs::Pose generateRandomPose(float radius, geometry_msgs::Pose reference_pose) const;
    bool isInMap(const geometry_msgs::Point &position) const;
    geometry_msgs::Point getMapCenter() const;
    bool isValidGoal(const geometry_msgs::Pose &goal, double &path_length);
    double computePlanLength(std::vector<geometry_msgs::PoseStamped> poses) const;

    // LOCALITZATION AND NAVIGATION
    bool moveRobot(const geometry_msgs::Pose &goal_pose);
    void move_baseDone(const actionlib::SimpleClientGoalState &state,
                       const move_base_msgs::MoveBaseResultConstPtr &result);
    void move_baseActive();
    bool updateRobotPose();
};

ExplorationBase::ExplorationBase(const std::string &_name) : Node(_name)
    : robot_status_(1),
      num_cells_(0),
      num_goals_sent_(0),
      num_goals_ko_(0),
      distance_traveled_(0),
      angle_traveled_(0),
      begin_exploration_(this->now()),
      cells_explored_(0),
      exploration_ended_(false),
      exploration_started_(false),
      got_map_(false),
      action_move_base_("move_base", true)
{
    // subscribers
    map_sub_ = nh_.subscribe("/map", 1, &ExplorationBase::mapCallback, this);
    frontiers_sub_ = nh_.subscribe("/frontiers", 1, &ExplorationBase::frontiersCallback, this);

    // services
    get_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");

    // Get parameters from ROS
    nh_.param<std::string>("results_file", results_file_, "~/exploration_results.csv");
    nh_.param<std::string>("scene", scene_, "undefined");

    // substitute '~' by home path
    auto found = results_file_.find('~');
    if (found != std::string::npos)
    {
        if (getenv("HOME") == NULL)
        {
            printf(
                "ERROR - ExplorationBase: Parameter 'results_file_' has '~' but 'HOME' environment variable is not "
                "defined.\n");
            return;
        }
        auto home_dir = std::string(getenv("HOME"));
        results_file_.replace(found, 1, home_dir + "/");
    }

    // check that file can be opened
    std::ofstream outfile(results_file_,
                          std::ofstream::out | std::ofstream::app); // append instead of overwrite
    if (not outfile.is_open())
    {
        printf("ERROR - ExplorationBase: couldn't open/write the results file %s\n", results_file_.c_str());
        perror("error message:");
        return;
    }

    srand((unsigned)time(NULL));
}

ExplorationBase::~ExplorationBase()
{
    // print file if ended by user or error
    if (not exploration_ended_)
        finish();
}

void ExplorationBase::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_ = *msg;

    // compute explored cells
    cells_explored_ = 0;
    for (int i = 0; i < map_.data.size(); ++i)
        if (map_.data[i] != -1)
            cells_explored_++;

    // ack
    got_map_ = true;
}

void ExplorationBase::frontiersCallback(const upc_mrn::Frontiers::ConstPtr &msg)
{
    frontiers_msg_ = *msg;

    // Until first forntiers message received, exploration doesn't start
    exploration_started_ = true;

    // If there is not frontiers, exploration ended!
    if (frontiers_msg_.frontiers.empty())
        exploration_ended_ = true;
}

geometry_msgs::Pose ExplorationBase::decideGoalBase()
{
    geometry_msgs::Pose g = decideGoal();

    // If the (for any reason) the decided goal is not a valid goal, generate a random goal in the surroundings
    // Iteratively increase the radius until we obtain a valid goal
    double radius = 0.1;
    double path_length;
    while (not isValidGoal(g, path_length))
    {
        printf(
            "!!! Goal provided by decideGoal() is not valid [%f, %f, %f].\nGenerating random goal with radius %f...\n",
            g.position.x,
            g.position.y,
            tf::getYaw(g.orientation),
            radius);
        g = generateRandomPose(radius, g);

        // Increase radius 5cm (limit the radius size to map size)
        if (radius < map_.info.height * map_.info.resolution or radius < map_.info.width * map_.info.resolution)
            radius += 0.05;
    }
    return g;
}

void ExplorationBase::loop()
{
    // NOT STARTED
    if (not exploration_started_ || not got_map_)
        return;

    if (updateRobotPose())
    {
        if (replan())
        {
            target_goal_ = decideGoalBase();
            moveRobot(target_goal_);
        }
    }
    else
        ROS_WARN("ExplorationBase::loop(): Couldn't get robot position!");

    // END
    if (exploration_ended_)
        finish();
}

void ExplorationBase::finish()
{
    bool err = false;
    double t = (ros::Time::now() - begin_exploration_).toSec();
    int t_min = int(t) / 60;
    int t_sec = int(t) % 60;

    // print stats
    printf("//////////////////////////////////////////////\n");
    printf("////////// EXPLORATION FINISHED! /////////////\n");
    printf("//////////////////////////////////////////////\n");
    printf("\tAlgorithm name: %s\n", this->get_name());
    printf("\tAlgorithm variant: %s\n", algorithm_variant_.c_str());
    printf("\tSent %i goals (cancelled %i)\n", num_goals_sent_, num_goals_ko_);
    printf("\tDistance traveled %.2f m\n", distance_traveled_);
    printf("\tAngle turned %.2f m\n", angle_traveled_);
    printf("\tDuration %2.2i:%2.2i min\n", t_min, t_sec);
    printf("\tExplored %.2f m^2 (%d cells)\n\n",
           cells_explored_ * map_.info.resolution * map_.info.resolution,
           cells_explored_);
    printf("!!!!!! If you want to save the map, remember:\n\trosrun map_server map_saver -f my_map_name\n\n");

    // store stats in CSV
    printf("Storing results in file: %s\n", results_file_.c_str());
    std::ofstream outfile(results_file_,
                          std::ofstream::out | std::ofstream::app); // append instead of overwrite
    if (outfile.is_open())
    {
        // initialize file with header
        if (outfile.tellp() == 0)
        {
            outfile << "#alg name, alg variant, date_time, scene, distance traveled (m), angle turned (rad), duration (s), cells "
                       "explored\n";
        }

        // current date and time on the current system
        time_t now = time(0);
        char *date_time = ctime(&now); // convert now to string form
        if (date_time[strlen(date_time) - 1] == '\n')
            date_time[strlen(date_time) - 1] = '\0'; // remove line break at the end

        outfile << this->get_name() << ", "   //
                << algorithm_variant_ << ", " //
                << date_time << ", "          //
                << scene_ << ", "             //
                << distance_traveled_ << ", " //
                << angle_traveled_ << ", "    //
                << t << ", "                  //
                << cells_explored_ << "\n";   //

        outfile.close();
    }
    else
    {
        printf("ExplorationBase::finish(): couldn't open/write the results file %s\n", results_file_.c_str());
        perror("error message:");
    }

    ros::shutdown();
}

///// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////
geometry_msgs::Pose ExplorationBase::generateRandomPose(float radius, geometry_msgs::Pose reference_pose) const
{
    geometry_msgs::Pose goal_pose;

    if (radius <= 0)
    {
        ROS_WARN("ExplorationBase::generateRandomPose(): radius must be > 0. Changed to 1");
        radius = 1;
    }

    // Check if reference_pose inside map
    if (not isInMap(reference_pose.position))
    {
        ROS_WARN(
            "ExplorationBase::generateRandomPose(): reference position is not inside the map. Changed to map center");
        reference_pose.position = getMapCenter();
    }

    // If radius is higher than map size, reference is center of the map
    if (radius >= map_.info.height * map_.info.resolution and radius >= map_.info.width * map_.info.resolution)
    {
        reference_pose.position = getMapCenter();
    }

    // srand((unsigned)time(NULL));
    float rand_yaw = 2 * M_PI * (rand() % 100) / 100.0;
    float rand_r = radius * (rand() % 100) / 100.0;

    goal_pose.position.x = reference_pose.position.x + rand_r * cos(rand_yaw);
    goal_pose.position.y = reference_pose.position.y + rand_r * sin(rand_yaw);
    goal_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(reference_pose.orientation) + rand_yaw);

    return goal_pose;
}

bool ExplorationBase::isInMap(const geometry_msgs::Point &position) const
{
    auto dx = position.x - map_.info.origin.position.x;
    auto dy = position.y - map_.info.origin.position.y;
    auto map_th = tf::getYaw(map_.info.origin.orientation);

    auto dx_map = cos(-map_th) * dx - sin(-map_th) * dy;
    auto dy_map = sin(-map_th) * dx + cos(-map_th) * dy;

    return dx_map >= 0 and dy_map >= 0 and dx_map <= map_.info.width * map_.info.resolution and
           dy_map <= map_.info.height * map_.info.resolution;
}

geometry_msgs::Point ExplorationBase::getMapCenter() const
{
    geometry_msgs::Point center;

    auto map_th = tf::getYaw(map_.info.origin.orientation);
    center.x = map_.info.origin.position.x + 0.5 * cos(map_th) * map_.info.width * map_.info.resolution -
               0.5 * sin(map_th) * map_.info.height * map_.info.resolution;
    center.y = map_.info.origin.position.y + 0.5 * sin(map_th) * map_.info.width * map_.info.resolution +
               0.5 * cos(map_th) * map_.info.height * map_.info.resolution;

    return center;
}

bool ExplorationBase::isValidGoal(const geometry_msgs::Pose &goal, double &path_length)
{
    if (not isInMap(goal.position))
        return false;

    nav_msgs::GetPlan get_plan_srv;
    get_plan_srv.request.start.header.stamp = ros::Time::now();
    get_plan_srv.request.start.header.frame_id = "map";
    get_plan_srv.request.start.pose = robot_pose_;

    get_plan_srv.request.goal.header.stamp = ros::Time::now();
    get_plan_srv.request.goal.header.frame_id = "map";
    get_plan_srv.request.goal.pose = goal;
    if (get_plan_client_.call(get_plan_srv))
    {
        if (get_plan_srv.response.plan.poses.size() != 0)
        {
            path_length = computePlanLength(get_plan_srv.response.plan.poses);
            ROS_DEBUG("Goal Valid! Path total distance: %f m", path_length);
            return true;
        }
    }
    else
        ROS_ERROR("ExplorationBase::isValidGoal(): Failed to call service get_plan");

    return false;
}

double ExplorationBase::computePlanLength(std::vector<geometry_msgs::PoseStamped> poses) const
{
    double length = 0.0;
    if (poses.size() >= 1)
    {
        for (unsigned int i = 1; i < poses.size(); i++)
        {
            double x1 = poses[i - 1].pose.position.x;
            double y1 = poses[i - 1].pose.position.y;
            double x2 = poses[i].pose.position.x;
            double y2 = poses[i].pose.position.y;
            double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            length += d;
        }
    }
    return length;
}

// NAVIGATION FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////
// moveRobot: send a goal to the robot
bool ExplorationBase::moveRobot(const geometry_msgs::Pose &goal_pose)
{
    // wait action server
    if (!action_move_base_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("ExplorationBase::moveRobot(): Waiting for the move_base action server to come up");
        return false;
    }

    // fill frame_id and stamp
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;

    num_goals_sent_++;
    ROS_DEBUG("ExplorationBase::moveRobot(): Sending Goal #%d: x=%4.2f, y=%4.2f, yaw=%4.2f in frame_id=%s",
              num_goals_sent_,
              goal.target_pose.pose.position.x,
              goal.target_pose.pose.position.y,
              tf::getYaw(goal.target_pose.pose.orientation),
              goal.target_pose.header.frame_id.c_str());

    ros::Duration t = ros::Time::now() - begin_exploration_;
    int elapsed_time_minutes = int(t.toSec()) / 60;
    int elapsed_time_seconds = int(t.toSec()) % 60;

    printf(
        ">>> Exploration status:\n\tSent %d goals (cancelled %d). Distance traveled %.2f m. Angle turned %.1f. "
        "Duration: %2.2i:%2.2i min. Explored %.2f m^2 (%d cells)\n",
        num_goals_sent_,
        num_goals_ko_,
        distance_traveled_,
        angle_traveled_,
        elapsed_time_minutes,
        elapsed_time_seconds,
        cells_explored_ * map_.info.resolution * map_.info.resolution,
        cells_explored_);

    action_move_base_.sendGoal(
        goal,
        boost::bind(&ExplorationBase::move_baseDone, this, _1, _2),
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleFeedbackCallback());
    robot_status_ = 0; // moving
    time_target_goal_ = ros::Time::now();
    return true;
}

// move_baseDone: is called when the robot reaches a goal or is cancelled for some reason
void ExplorationBase::move_baseDone(const actionlib::SimpleClientGoalState &state,
                                    const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        robot_status_ = 1; // success
    }
    else
    {
        robot_status_ = 2; // error
        num_goals_ko_++;
    }
}

// Compute the current robot pose via TF
bool ExplorationBase::updateRobotPose()
{
    geometry_msgs::Pose prev_robot_pose = robot_pose_;

    tf::StampedTransform transform;
    ros::Time target_time = ros::Time(0); // ros::Time::now();
    std::string source_frame = "/map";
    std::string target_frame = "/base_footprint";
    try
    {
        if (listener_.waitForTransform(source_frame, target_frame, target_time, ros::Duration(5.0)))
            listener_.lookupTransform(source_frame, target_frame, target_time, transform);
        else
        {
            ROS_ERROR("ExplorationBase::updateRobotPose(): no transform between frames %s and %s",
                      source_frame.c_str(),
                      target_frame.c_str());
            return false;
        }
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR(
            "ExplorationBase::updateRobotPose(): could not compute transform between frames %s and %s with error: %s",
            source_frame.c_str(),
            target_frame.c_str(),
            ex.what());
        return false;
    }
    robot_pose_.position.x = transform.getOrigin().x();
    robot_pose_.position.y = transform.getOrigin().y();
    robot_pose_.position.z = 0.0;
    robot_pose_.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));

    if (prev_robot_pose.position.x != 0 and prev_robot_pose.position.y != 0)
    {
        float distance_incr = std::sqrt(std::pow(prev_robot_pose.position.x - robot_pose_.position.x, 2) +
                                        std::pow(prev_robot_pose.position.y - robot_pose_.position.y, 2));
        distance_traveled_ += fabs(distance_incr);

        angle_traveled_ += fabs(tf::getYaw(prev_robot_pose.orientation) - tf::getYaw(robot_pose_.orientation));
    }

    return true;
}
