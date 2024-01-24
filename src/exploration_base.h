#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "upc_mrn/msg/frontiers.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// Action example: https://github.com/ros2/examples/blob/humble/rclcpp/actions/minimal_action_client/member_functions.cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "tf2/utils.h"                             // getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // toMsg
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class ExplorationBase : public rclcpp::Node
{
public:
    using NavToPose = nav2_msgs::action::NavigateToPose;
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

protected:
    // ------------------ ATTRIBUTES ------------------
    std::string algorithm_variant_; // variant of the algorithm (in case you need it)
    std::string results_file_;      // name of the results file (csv)
    std::string world_;             // name of the world

    int robot_status_;                    // 0: moving, 1: goal reached, 2: failed to reach goal
    geometry_msgs::msg::Pose robot_pose_; // current robot pose
    geometry_msgs::msg::Pose goal_;       // last goal sent
    rclcpp::Time goal_stamp_;             // time when last goal was sent
    double goal_time_;                    // seconds since last goal was sent
    double goal_distance_;                // remaining distance to reach the last goal

    // Subscribers/publishers
    rclcpp::Subscription<upc_mrn::msg::Frontiers>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_new_pub_;
    nav_msgs::msg::OccupancyGrid map_;
    upc_mrn::msg::Frontiers frontiers_msg_;

    // actions
    rclcpp_action::Client<NavToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // GLOBAL STATS
    int cells_explored_;             // explored cells (free or obstacle)
    int num_goals_sent_;             // total goals sent
    int num_goals_ko_;               // total goals cancelled
    double distance_traveled_;       // total absolute distance traveled during exploration (m)
    double angle_traveled_;          // total absolute angle turned during exploration (rad)
    rclcpp::Time begin_exploration_; // used to compute the total exploration time

    // Flags
    bool exploration_ended_;   // flag to finish exploration
    bool exploration_started_; // flag to start exploration
    bool got_map_;             // flag to ack the map reception

    // ------------------ METHODS ------------------
public:
    ExplorationBase(const std::string &_name);
    ~ExplorationBase();
    void loop();

protected:
    // CALLBACKS (inputs)
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void frontiersCallback(const upc_mrn::msg::Frontiers &msg);

    // EXPLORATION ALGORITHM MAIN FUNCTIONS
    // Functions to be implemented in derived class
    virtual bool replan() = 0;
    virtual geometry_msgs::msg::Pose decideGoal() = 0;
    // Base class functions
    virtual geometry_msgs::msg::Pose decideGoalBase();
    void finish();

    // AUXILIARY
    geometry_msgs::msg::Pose generateRandomPose(float radius, geometry_msgs::msg::Pose reference_pose);
    bool isValidGoal(const geometry_msgs::msg::Pose &goal, double &path_distance);
    bool isInMap(const geometry_msgs::msg::Point &position) const;
    bool isFree(const geometry_msgs::msg::Point &position) const;
    geometry_msgs::msg::Point getMapCenter() const;
    int floor0(const float &value) const;

    // NAVIGATION
    // send goal
    bool sendGoal(const geometry_msgs::msg::Pose &goal_pose);
    void goalResponseCallback(const GoalHandleNavToPose::SharedPtr &goal_handle);
    void goalFeedbackCallback(const GoalHandleNavToPose::SharedPtr &goal_handle,
                              const std::shared_ptr<const NavToPose::Feedback> feedback);
    void goalResultCallback(const GoalHandleNavToPose::WrappedResult &result);
    // compute path
    double computePathLength(const geometry_msgs::msg::Pose &goal_pose);

    // TF
    bool updateRobotPose();
};

ExplorationBase::ExplorationBase(const std::string &_name)
    : Node(_name),
      robot_status_(1),
      cells_explored_(0),
      num_goals_sent_(0),
      num_goals_ko_(0),
      distance_traveled_(0),
      angle_traveled_(0),
      begin_exploration_(this->now()),
      exploration_ended_(false),
      exploration_started_(false),
      got_map_(false)
{
    // subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&ExplorationBase::mapCallback, this, std::placeholders::_1));
    frontiers_sub_ = this->create_subscription<upc_mrn::msg::Frontiers>(
        "/frontiers", 1, std::bind(&ExplorationBase::frontiersCallback, this, std::placeholders::_1));

    // Get parameters from ROS
    declare_parameter("results_file", "~/exploration_results.csv");
    get_parameter("results_file", results_file_);
    declare_parameter("world", "undefined");
    get_parameter("world", world_);
    declare_parameter("algorithm_variant", "");
    get_parameter("algorithm_variant", algorithm_variant_);
    int node_period;
    declare_parameter("node_period_ms", 500);
    get_parameter("node_period", node_period);

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

    // action of nav2
    nav_to_pose_client_ = rclcpp_action::create_client<NavToPose>(this, "bt_navigator/navigate_to_pose");

    // tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timer to periodically call loop()
    timer_ = this->create_wall_timer(std::chrono::milliseconds(node_period), std::bind(&MinimalPublisher::loop, this));

    // seed for random
    srand((unsigned)time(NULL));
}

ExplorationBase::~ExplorationBase()
{
    // print file if ended by user or error
    if (not exploration_ended_)
        finish();
}

void ExplorationBase::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    // store map
    map_ = msg;

    // compute explored cells
    cells_explored_ = 0;
    for (unsigned int i = 0; i < map_.data.size(); ++i)
        if (map_.data[i] != -1)
            cells_explored_++;

    // flag
    got_map_ = true;
}

void ExplorationBase::frontiersCallback(const upc_mrn::msg::Frontiers &msg)
{
    frontiers_msg_ = msg;

    // Until first forntiers message received, exploration doesn't start
    exploration_started_ = true;

    // If there is not frontiers, exploration ended!
    if (frontiers_msg_.frontiers.empty())
        exploration_ended_ = true;
}

geometry_msgs::msg::Pose ExplorationBase::decideGoalBase()
{
    geometry_msgs::msg::Pose g = decideGoal();

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
            tf2::getYaw(g.orientation),
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
            sendGoal(decideGoalBase());
        }
    }
    else
        RCLCPP_WARN(this->get_logger(), "ExplorationBase::loop(): Couldn't get robot position!");

    // END
    if (exploration_ended_)
        finish();
}

void ExplorationBase::finish()
{
    double t = (this->now() - begin_exploration_).seconds();
    int t_min = int(t) / 60;
    int t_sec = int(t) % 60;

    // print stats
    printf("//////////////////////////////////////////////////////\n");
    if (exploration_ended_)
        printf("//////////////// EXPLORATION FINISHED ////////////////\n");
    else
        printf("// EXPLORATION ENDED BEFORE EXPLORING THE WHOLE MAP //\n");
    printf("//////////////////////////////////////////////////////\n");
    printf("\tAlgorithm name: %s\n", this->get_name());
    printf("\tAlgorithm variant: %s\n", algorithm_variant_.c_str());
    printf("\tWorld: %s\n", world_.c_str());
    printf("\tSent %i goals (aborted %i)\n", num_goals_sent_, num_goals_ko_);
    printf("\tDistance traveled %.2f m\n", distance_traveled_);
    printf("\tAngle turned %.2f m\n", angle_traveled_);
    printf("\tDuration %2.2i:%2.2i min\n", t_min, t_sec);
    printf("\tExplored %.2f m^2 (%d cells)\n\n",
           cells_explored_ * map_.info.resolution * map_.info.resolution,
           cells_explored_);

    // store stats in CSV
    printf("Storing results in file: %s\n", results_file_.c_str());
    std::ofstream outfile(results_file_,
                          std::ofstream::out | std::ofstream::app); // append instead of overwrite
    if (outfile.is_open())
    {
        // initialize file with header
        if (outfile.tellp() == 0)
        {
            outfile << "#DATE_TIME, "
                    << "FINISHED, "
                    << "ALGORITHM NAME, "
                    << "VARIANT, "
                    << "WORLD, "
                    << "GOALS SENT, "
                    << "GOALS ABORTED, "
                    << "DISTANCE TRAVELED (m), "
                    << "ANGLE TURNED (rad), "
                    << "DURATION (s), "
                    << "CELLS EXPLORED\n";
        }

        // current date and time on the current system
        time_t now = time(0);
        char *date_time = ctime(&now); // convert now to string form
        if (date_time[strlen(date_time) - 1] == '\n')
            date_time[strlen(date_time) - 1] = '\0'; // remove line break at the end

        outfile << date_time << ", "          //
                << exploration_ended_ << ", " //
                << this->get_name() << ", "   //
                << algorithm_variant_ << ", " //
                << world_ << ", "             //
                << num_goals_sent_ << ", "    //
                << num_goals_ko_ << ", "      //
                << world_ << ", "             //
                << distance_traveled_ << ", " //
                << angle_traveled_ << ", "    //
                << t << ", "                  //
                << cells_explored_ << "\n";   //

        outfile.close();
    }
    else
    {
        printf("ExplorationBase::finish(): couldn't open/write the results file %s\n", results_file_.c_str());
    }

    rclcpp::shutdown();
}

///// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////
geometry_msgs::msg::Pose ExplorationBase::generateRandomPose(float radius, geometry_msgs::msg::Pose reference_pose)
{
    geometry_msgs::msg::Pose goal_pose;

    if (radius <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "ExplorationBase::generateRandomPose(): radius must be > 0. Changed to 1");
        radius = 1;
    }

    // Check if reference_pose inside map
    if (not isInMap(reference_pose.position))
    {
        RCLCPP_WARN(this->get_logger(),
                    "ExplorationBase::generateRandomPose(): reference position is not inside the map. Changed to map center");
        reference_pose.position = getMapCenter();
    }

    // If radius is higher than map size, reference is center of the map
    if (radius >= map_.info.height * map_.info.resolution and radius >= map_.info.width * map_.info.resolution)
    {
        reference_pose.position = getMapCenter();
    }

    // Random radius and angle
    float rand_yaw = 2 * M_PI * (rand() % 100) / 100.0;
    float rand_r = radius * (rand() % 100) / 100.0;

    // Compose over reference pose
    goal_pose.position.x = reference_pose.position.x + rand_r * cos(rand_yaw);
    goal_pose.position.y = reference_pose.position.y + rand_r * sin(rand_yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(reference_pose.orientation) + rand_yaw);
    goal_pose.orientation = tf2::toMsg(q);

    return goal_pose;
}

bool ExplorationBase::isValidGoal(const geometry_msgs::msg::Pose &goal, double &path_length)
{
    // not free (also returning false if out of map)
    if (not isFree(goal.position))
        return false;

    // return true if a path to that pose could be computed
    path_length = computePathLength(goal);
    return path_length >= 0;
}

bool ExplorationBase::isInMap(const geometry_msgs::msg::Point &position) const
{
    auto dx = position.x - map_.info.origin.position.x;
    auto dy = position.y - map_.info.origin.position.y;
    auto map_th = tf2::getYaw(map_.info.origin.orientation);

    auto dx_map = cos(-map_th) * dx - sin(-map_th) * dy;
    auto dy_map = sin(-map_th) * dx + cos(-map_th) * dy;

    return dx_map >= 0 and dy_map >= 0 and dx_map <= map_.info.width * map_.info.resolution and
           dy_map <= map_.info.height * map_.info.resolution;
}

bool ExplorationBase::isFree(const geometry_msgs::msg::Point &point) const
{
    if (not isInMap(point))
        return false;

    // cell
    int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
    int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
    int cell = x_cell + (y_cell)*map_.info.width;

    // return if free
    return map_.data[cell] == 0;
}

int ExplorationBase::floor0(const float &value) const
{
    if (value < 0.0)
        return ceil(value);
    else
        return floor(value);
}

geometry_msgs::msg::Point ExplorationBase::getMapCenter() const
{
    geometry_msgs::msg::Point center;

    auto map_th = tf2::getYaw(map_.info.origin.orientation);
    center.x = map_.info.origin.position.x + 0.5 * cos(map_th) * map_.info.width * map_.info.resolution -
               0.5 * sin(map_th) * map_.info.height * map_.info.resolution;
    center.y = map_.info.origin.position.y + 0.5 * sin(map_th) * map_.info.width * map_.info.resolution +
               0.5 * cos(map_th) * map_.info.height * map_.info.resolution;

    return center;
}

// NAVIGATION FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////
bool ExplorationBase::sendGoal(const geometry_msgs::msg::Pose &goal_pose)
{
    using namespace std::placeholders;

    if (not nav_to_pose_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }

    // Fill goal msg
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose = goal_pose;

    // reset goal stats
    goal_ = goal_pose;
    goal_distance_ = -1;
    goal_time_ = 0;
    goal_stamp_ = goal.pose.header.stamp;
    num_goals_sent_++;

    // print
    RCLCPP_INFO(this->get_logger(), "ExplorationBase::moveRobot(): Sending Goal #%d: x=%4.2f, y=%4.2f, yaw=%4.2f in frame_id=%s",
                num_goals_sent_,
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
                tf2::getYaw(goal.pose.pose.orientation),
                goal.pose.header.frame_id.c_str());

    rclcpp::Duration t = this->now() - begin_exploration_;
    int elapsed_time_minutes = int(t.seconds()) / 60;
    int elapsed_time_seconds = int(t.seconds()) % 60;

    printf(
        ">>> Exploration status:\n\tSent %d goals (aborted %d). Distance traveled %.2f m. Angle turned %.1f. "
        "Duration: %2.2i:%2.2i min. Explored %.2f m^2 (%d cells)\n",
        num_goals_sent_,
        num_goals_ko_,
        distance_traveled_,
        angle_traveled_,
        elapsed_time_minutes,
        elapsed_time_seconds,
        cells_explored_ * map_.info.resolution * map_.info.resolution,
        cells_explored_);

    // Send goal
    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ExplorationBase::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ExplorationBase::goalFeedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ExplorationBase::goalResultCallback, this, _1);
    nav_to_pose_client_->async_send_goal(goal, send_goal_options);

    return true;
}

void ExplorationBase::goalResponseCallback(const GoalHandleNavToPose::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by nav_to_pose action server");
        robot_status_ = 2; // cancelled
        num_goals_ko_++;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by nav_to_pose action server, waiting for result");
        robot_status_ = 0; // moving
    }
}

void ExplorationBase::goalFeedbackCallback(const GoalHandleNavToPose::SharedPtr &goal_handle,
                                           const std::shared_ptr<const NavToPose::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback from nav_to_pose action");

    goal_distance_ = feedback->distance_remaining;
    goal_time_ = rclcpp::Duration(feedback->navigation_time).seconds();
    RCLCPP_INFO(this->get_logger(), "feedback->navigation_time = %f (should be %f)", goal_time_, (this->now() - goal_stamp_).seconds());
}

void ExplorationBase::goalResultCallback(const GoalHandleNavToPose::WrappedResult &result)
{
    RCLCPP_INFO(this->get_logger(), "Received result from nav_to_pose action");

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_ERROR(this->get_logger(), "Result from nav_to_pose: Goal was reached");
        robot_status_ = 1; // reached
        return;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Result from nav_to_pose: Goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Result from nav_to_pose: Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Result from nav_to_pose: Unknown result code");
        break;
    }
    robot_status_ = 2; // aborted/cancelled
    num_goals_ko_++;
}

double ExplorationBase::computePathLength(const geometry_msgs::msg::Pose &goal_pose)
{
    if (!compute_path_client_->wait_for_action_server(std::chrono::milliseconds(50)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server compute_path_to_pose not available after waiting 50ms");
        return -1;
    }

    // Populate a goal
    RCLCPP_INFO(this->get_logger(), "Sending goal to compute_path_to_pose");
    auto goal_msg = ComputePath::Goal();
    goal_msg.goal.pose = goal_pose;
    goal_msg.use_start = false; // use current robot pose as path start

    // Wait 50ms for the server to accept the goal
    auto goal_handle_future = compute_path_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future, std::chrono::milliseconds(50)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: send goal call failed :(");
        return -1;
    }

    GoalHandleComputePath::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was rejected by the server");
        return -1;
    }

    // Wait 50ms more for the server to be done with the goal
    auto result_future = compute_path_client_->async_get_result(goal_handle);
    RCLCPP_INFO(this->get_logger(), "compute_path_to_pose: Waiting for result");
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future, std::chrono::milliseconds(50)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: get result call failed :(");
        return -1;
    }

    // get the result
    GoalHandleComputePath::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was aborted");
        return -1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was canceled");
        return -1;
    default:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Unknown result code");
        return -1;
    }

    // compute path length
    double length = 0.0;
    if (wrapped_result.result->path.poses.size() >= 1)
    {
        for (unsigned int i = 1; i < wrapped_result.result->path.poses.size(); i++)
        {
            double &x1 = wrapped_result.result->path.poses[i - 1].pose.position.x;
            double &y1 = wrapped_result.result->path.poses[i - 1].pose.position.y;
            double &x2 = wrapped_result.result->path.poses[i].pose.position.x;
            double &y2 = wrapped_result.result->path.poses[i].pose.position.y;
            double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            length += d;
        }
    }
    return length;
}

// Compute the current robot pose via TF
bool ExplorationBase::updateRobotPose()
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose prev_robot_pose = robot_pose_;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    std::string source_frame = "/map";
    std::string target_frame = "/base_link";
    try
    {
        t = tf_buffer_->lookupTransform(
            source_frame, target_frame,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            target_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }

    // Convert from transform to pose
    robot_pose_.position.x = t.transform.translation.x;
    robot_pose_.position.y = t.transform.translation.y;
    robot_pose_.position.z = 0.0;
    robot_pose_.orientation = t.transform.rotation;

    if (prev_robot_pose.position.x != 0 and prev_robot_pose.position.y != 0)
    {
        float distance_incr = std::sqrt(std::pow(prev_robot_pose.position.x - robot_pose_.position.x, 2) +
                                        std::pow(prev_robot_pose.position.y - robot_pose_.position.y, 2));
        distance_traveled_ += fabs(distance_incr);

        angle_traveled_ += fabs(tf2::getYaw(prev_robot_pose.orientation) - tf2::getYaw(robot_pose_.orientation));
    }

    return true;
}