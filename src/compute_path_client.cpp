#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

// Action example: https://github.com/ros2/examples/blob/humble/rclcpp/actions/minimal_action_client/member_functions.cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "tf2/utils.h"                             // getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // toMsg

class ComputePathClient : public rclcpp::Node
{
public:
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

protected:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid map_;
    bool got_map_; // flag to ack the map reception

    // actions
    rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

public:
    ComputePathClient();
    ~ComputePathClient(){};
    void sendGoal();

protected:
    // CALLBACKS (inputs)
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    geometry_msgs::msg::Pose generateRandomPose();
};

ComputePathClient::ComputePathClient()
    : Node("compute_path_client"),
      got_map_(false)
{
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // subscribers
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&ComputePathClient::mapCallback, this, std::placeholders::_1), options);

    // Get parameters from ROS
    int node_period;
    declare_parameter("node_period_ms", 500);
    get_parameter("node_period_ms", node_period);

    // action of nav2
    compute_path_client_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose", callback_group_);

    // timer to periodically call loop()
    timer_ = this->create_wall_timer(std::chrono::milliseconds(node_period), std::bind(&ComputePathClient::sendGoal, this), callback_group_);

    // seed for random
    srand((unsigned)time(NULL));
}

void ComputePathClient::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    RCLCPP_INFO(this->get_logger(), "ComputePathClient::mapCallback");
    map_ = msg;
    got_map_ = true;
}

///// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////
geometry_msgs::msg::Pose ComputePathClient::generateRandomPose()
{
    RCLCPP_INFO(this->get_logger(), "ComputePathClient::generateRandomPose");

    auto radius = map_.info.height * map_.info.resolution / 2;

    // reference center of the map
    auto map_th = tf2::getYaw(map_.info.origin.orientation);
    geometry_msgs::msg::Pose reference_pose;
    reference_pose.position.x = map_.info.origin.position.x + 0.5 * cos(map_th) * map_.info.width * map_.info.resolution -
                                0.5 * sin(map_th) * map_.info.height * map_.info.resolution;
    reference_pose.position.y = map_.info.origin.position.y + 0.5 * sin(map_th) * map_.info.width * map_.info.resolution +
                                0.5 * cos(map_th) * map_.info.height * map_.info.resolution;

    // Random radius and angle
    float rand_yaw = 2 * M_PI * (rand() % 100) / 100.0;
    float rand_r = radius * (rand() % 100) / 100.0;

    // Compose over reference pose
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = reference_pose.position.x + rand_r * cos(rand_yaw);
    goal_pose.position.y = reference_pose.position.y + rand_r * sin(rand_yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(reference_pose.orientation) + rand_yaw);
    goal_pose.orientation = tf2::toMsg(q);

    return goal_pose;
}

void ComputePathClient::sendGoal()
{
    if (not got_map_)
        return;

    RCLCPP_INFO(this->get_logger(), "ComputePathClient::sendGoal");

    if (!compute_path_client_->wait_for_action_server(std::chrono::milliseconds(50)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server compute_path_to_pose not available after waiting 50ms");
        return;
    }

    if (!compute_path_client_->action_server_is_ready())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server compute_path_to_pose not ready");
        return;
    }

    // Populate a goal
    auto goal_msg = ComputePath::Goal();
    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.pose = generateRandomPose();
    goal_msg.use_start = false; // use current robot pose as path start
    RCLCPP_INFO(this->get_logger(), "Sending goal to compute_path_to_pose: x: %f, y: %f", goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y);

    // Wait 50ms for the server to accept the goal
    auto goal_handle_future = compute_path_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Goal sent");

    if (goal_handle_future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: send goal call failed :(");
        return;
    }

    auto goal_handle = goal_handle_future.get();
    std::cout << "future_valid: " << goal_handle_future.valid() << std::endl;

    RCLCPP_INFO(this->get_logger(), "compute_path_to_pose: got goal_handle.");
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was rejected by the server");
        return;
    }

    // Wait 50ms more for the server to be done with the goal
    RCLCPP_INFO(this->get_logger(), "compute_path_to_pose: Gettint result future");
    auto result_future = compute_path_client_->async_get_result(goal_handle);
    RCLCPP_INFO(this->get_logger(), "compute_path_to_pose: Waiting for result");

    if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: get result call failed :(");
        return;
    }

    // get the result
    GoalHandleComputePath::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Unknown result code");
        return;
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

    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!! compute_path_to_pose: length = %f", length);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ComputePathClient>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    return 0;
}