#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // toMsg

class SafetyNode : public rclcpp::Node
{
public:
  // Constructor
  SafetyNode();
private:

  // ----- Attributes -----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_safe_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::mutex mutex_scan_;

  double t_safe_, v_max_, w_max_, robot_radius_;
  double sensor_x_, sensor_y_, sensor_z_, sensor_yaw_;
  std::vector<double> scan_x_, scan_y_;
  rclcpp::Time scan_ts_;

  std_msgs::msg::ColorRGBA red;
  std_msgs::msg::ColorRGBA green;
  
  geometry_msgs::msg::Twist cmdvel_safe_msg_;

  // ----- Methods -----
  void laserCallback(const sensor_msgs::msg::LaserScan &laser_msg);
  void cmdvelCallback(const geometry_msgs::msg::Twist &msg);
  geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll,
                                                     double pitch,
                                                     double yaw);
  void publish_marker(const double& x,
                      const double& y,
                      const std::string&         frame_id,
                      const std::string&         ns,
                      const int&                 id,
                      const std_msgs::msg::ColorRGBA& color,
                      const rclcpp::Time &ts);
  bool SameSign(double x, double y);   
};


SafetyNode::SafetyNode() : Node("safety_node", rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true))
{
  // initialization
  v_max_ = 1e9;
  w_max_ = 1e9;  // just initialization
  red.r   = 1.0;
  red.a   = 0.5;
  green.g = 1.0;
  green.a = 0.5;

  // Sensor frame expressed in robot base
  sensor_x_   = 0.004;
  sensor_y_   = 0.0;
  sensor_z_   = 0.139;
  sensor_yaw_ = 1.571;

  // Declare parameters with default values
  declare_parameter("t_safe", 1.0);       // default 1s
  declare_parameter("robot_radius", 0.2); // default 20cm

  // load parameters
  get_parameter("t_safe", t_safe_);
  get_parameter("robot_radius", robot_radius_);

  RCLCPP_INFO(get_logger(), "SafetyNode constructor! Parameters: t_safe = %fs, robot_radius = %fs", t_safe_, robot_radius_);

  // store scan time stamp
  scan_ts_ = this->get_clock()->now();

  // publisher and subscribers
  cmdvel_safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  marker_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("/safety_marker", 1);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&SafetyNode::laserCallback, this, std::placeholders::_1));
  cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_teleop", 10, std::bind(&SafetyNode::cmdvelCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::Quaternion SafetyNode::quaternion_from_rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

void SafetyNode::publish_marker(const double& x,
  const double& y,
  const std::string&         frame_id,
  const std::string&         ns,
  const int&                 id,
  const std_msgs::msg::ColorRGBA& color,
  const rclcpp::Time &ts)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = frame_id;
  marker.header.stamp       = ts;
  marker.ns                 = ns;
  marker.id                 = id;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x    = x;
  marker.pose.position.y    = y;
  marker.pose.position.z    = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.2;
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.01;
  marker.color              = color;
  marker.lifetime           = rclcpp::Duration(0, 1e8);
  marker_pub_->publish(marker);
}

bool SafetyNode::SameSign(double x, double y)
{
    return (x >= 0) ^ (y < 0);
}

// Callback of laser scan
void SafetyNode::laserCallback(const sensor_msgs::msg::LaserScan &laser_msg)
{
  mutex_scan_.lock();

  scan_ts_ = laser_msg.header.stamp;

  // Transform to cartesian and express w.r.t. robot base
  int size = laser_msg.ranges.size();
  //ROS_INFO("Received scan ranges size=%d", size);
  scan_x_.resize(size, 0.0);
  scan_y_.resize(size, 0.0);

  // =================
  // TODO 1: Convert the scan from polar to cartesian coordinates
  //         and store it in scan_x_ and scan_y_ variables
  // =================

  float angle_min = laser_msg.angle_min;
  float angle_inc = laser_msg.angle_increment;
  
  for(int i=0; i<size; i++)
  {
    float angle = angle_min + i*angle_inc;
    scan_x_[i] = laser_msg.ranges[i] * cos(angle);
    scan_y_[i] = laser_msg.ranges[i] * sin(angle);
  }   

  // =================
  // END TODO 1


  // =================
  // TODO 2: Convert scan_x_ and scan_y_ coordinates to be expressed in the robot base link
  //         The pose of the laser frame in the robot base is already defined by the variables
  //         sensor_x_, sensor_y_, sensor_z_ and sensor_yaw_
  // =================

  for(int i=0; i<size; i++)
  {
    double x_i = scan_x_[i];
    double y_i = scan_y_[i];
    scan_x_[i] = x_i*cos(sensor_yaw_) - y_i*sin(sensor_yaw_) + sensor_x_;
    scan_y_[i] = x_i*sin(sensor_yaw_) + y_i*cos(sensor_yaw_) + sensor_y_;
  }

  // =================
  // END TODO 2

  mutex_scan_.unlock();

}

void SafetyNode::cmdvelCallback(const geometry_msgs::msg::Twist &msg)
{
  cmdvel_safe_msg_ = msg; // copy of the received message

  // Here we assume a circular robot, thus only the Vx is processed
  if (cmdvel_safe_msg_.linear.x != 0.0)
  {
    mutex_scan_.lock();
    std::vector<double> scan_x = scan_x_;
    std::vector<double> scan_y = scan_y_;
    rclcpp::Time scan_ts = scan_ts_;
    mutex_scan_.unlock();

    bool collision = false;
    int collision_idx = 0;

    // Initializations
    double px = 0.0;
    double py = 0.0;
    double obst_dist = 100.0;
    double dt = 0.1;
    int num_states = t_safe_/dt;

    // =================
    // TODO 3: 
    //         a) Build a loop to "propagate" the forward pose of the robot 
    //         using a simple kinematic model that uses the commanded (input) cmd_vel (vx).
    //
    //          px = px + vx*dt
    //
    //         "num_states" and "dt" are already set.
    //
    //         b) Check for collisions in each propagation step. For each point in (scan_x, scan_y) 
    //         check if the propagated position (px, py) is closer than the robot radius (robot_radius_).
    //
    //         If there's collision, set to TRUE the "collision" parameters and store the index of 
    //         the scan point that is closest to the propagated position in the "collision_idx" variable.
    //
    //         Help: A marker will be published in (scan_x[collision_idx], scan_y[collision_idx])
    //         to show you in rviz where you estimate the nearest collision.


    // =================
    
    double linear_vel_teleop_ = cmdvel_safe_msg_.linear.x;
    double angular_vel_teleop_ = cmdvel_safe_msg_.angular.z;
    double pth = 0.0;
    std::vector<double> pred_pose_x_vect(num_states, 0.0);  // Path vector of the robot w.r.t. base_link
    std::vector<double> pred_pose_y_vect(num_states, 0.0);

    for (int i = 1; i < num_states; i += 1){
      float vx = linear_vel_teleop_;
      float w = angular_vel_teleop_;
      
      pth += w * dt / 2;
      px += vx * dt * cos(pth);
      py += vx * dt * sin(pth);
      pth += w * dt / 2;

      // pred_pose_x_vect[i] = px;
      // pred_pose_y_vect[i] = py;

      for (unsigned int j = 0; j < scan_x.size(); j += 1) {
        obst_dist = std::sqrt(std::pow((scan_x[j]-px),2)+std::pow((scan_y[j]-py),2));
        if (robot_radius_ > obst_dist) {
          collision = true;
          collision_idx = j;
        }
      }
    }

    // END TODO 3

    // Visualization
    if (collision_idx != 0)
      publish_marker(scan_x[collision_idx], scan_y[collision_idx], "base_link", "collision", 1, red, scan_ts);

    // Manage velocity saturation if needed
    if  (collision && SameSign(cmdvel_safe_msg_.linear.x, scan_x[collision_idx]))
    {
      double collision_distance = scan_x[collision_idx] - robot_radius_;
      double v_max_abs = collision_distance / t_safe_;
      double ratio = v_max_abs / cmdvel_safe_msg_.linear.x;
      cmdvel_safe_msg_.linear.x = v_max_abs;
      cmdvel_safe_msg_.angular.z = ratio * cmdvel_safe_msg_.angular.z;
      RCLCPP_WARN(this->get_logger(), "Collision! Reducing Velocity! vx:%f wz:%f", cmdvel_safe_msg_.linear.x, cmdvel_safe_msg_.angular.z); 
    }

  } // cmdvel_safe_msg_.linear.x != 0.0

  // publish the safe velocity command
  cmdvel_safe_pub_->publish(cmdvel_safe_msg_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}