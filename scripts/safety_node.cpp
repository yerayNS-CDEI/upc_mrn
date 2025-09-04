#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

// Variable definitions
float pose_x;
float pose_y;
float pose_yaw;

float min_dist = 10e9;
float dist_act_col = 10e9;
float min_dist_i = 10e9;
double collision = 0;
float t_impact ;

// Class definition
class SafetyNode
{
  private:
    ros::Publisher  cmdvel_safe_pub_;
    ros::Subscriber cmdvel_sub_, laser_sub_, odom_sub_;
    double          t_safe_, v_max_, robot_radius_;
    double          robot_colision_side_x, robot_colision_side_y;
    double          w_max_, linear_vel_teleop_, angular_vel_teleop_;

    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    std_msgs::ColorRGBA blue;
    ros::Publisher      points_pub_;

  public:
    SafetyNode(ros::NodeHandle& nh);

  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void publish_marker(const std::vector<double>& x,
                        const std::vector<double>& y,
                        const int&                 subsampling,
                        const std::string&         frame_id,
                        const std::string&         ns,
                        const int&                 id,
                        const double&              z,
                        const std_msgs::ColorRGBA& color,
                        const ros::Time&           t);
};

SafetyNode::SafetyNode(ros::NodeHandle& nh)
{
    v_max_ = 1e9;  // just initialization

    // load values from ROS param with default values
    nh.param<double>("t_safe", t_safe_, 5.0);
    nh.param<double>("robot_radius", robot_radius_, 0.2);

    // Publishers and Subscribers Objects
    laser_sub_       = nh.subscribe("/scan", 1, &SafetyNode::scanCallback, this);
    cmdvel_sub_      = nh.subscribe("/cmd_vel_teleop", 1, &SafetyNode::cmdvelCallback, this);
    odom_sub_        = nh.subscribe("/odom", 1, &SafetyNode::odom_callback, this);
    cmdvel_safe_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    points_pub_      = nh.advertise<visualization_msgs::Marker>("/points", 1);

    // Color variables for markers
    this->red.r   = 1.0;
    this->green.g = 1.0;
    this->blue.b  = 1.0;
    this->blue.r  = 1.0;
    this->blue.g  = 1.0;
}

void SafetyNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();

    // print & visualize one of each n points
    int n_viz   = 1;        // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS
    int n_print = size / 4;  // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS

    // Preinitialization of the Collision binary variable
    collision = 0;

    // TODO 1: compute the maximum velocity allowed

    // Transform from polar to cartesian (Obstacle position w.r.t. robot frame)
    std::vector<double> x(size, 0.0);
    std::vector<double> y(size, 0.0);
    for (unsigned int i = 0; i < x.size(); i += 1)
      x[i] = msg->ranges[i] * cos(i*msg->angle_increment) - 0.064;
    for (unsigned int i = 0; i < y.size(); i += 1)
      y[i] = msg->ranges[i] * sin(i*msg->angle_increment);

    //Obstacle position w.r.t. world frame
    std::vector<double> x_obst(size, 0.0);
    std::vector<double> y_obst(size, 0.0);
    for (unsigned int i = 0; i < x.size(); i += 1)
     x_obst[i] = cos(pose_yaw)*x[i] - sin(pose_yaw)*y[i] + pose_x;
    for (unsigned int i = 0; i < y.size(); i += 1)
     y_obst[i] = sin(pose_yaw)*x[i] + cos(pose_yaw)*y[i] + pose_y;

     // Publishing obstacles contours to plot in RVIZ detected by the LIDAR LaserScan
     publish_marker(x_obst, y_obst, 4, "odom", "world", 1, 0.132, blue, msg->header.stamp);


    //Path Prediction
    double n_path = 25;  // Number of predicted points

    std::vector<double> pred_pose_x_vect(n_path, 0.0);  // Path vector of the robot w.r.t. world frame
    std::vector<double> pred_pose_y_vect(n_path, 0.0);
    float dt = 0.5;  // Time-step considered
    float pred_pose_x = pose_x;  // Predicted pose
    float pred_pose_y = pose_y;
    float pred_pose_yaw = pose_yaw;

      // Reinitialization of constants
    min_dist = 10e9;
    min_dist_i = 10;
    t_impact = 0;

    for (unsigned int i = 1; i < n_path; i += 1){
      pred_pose_x_vect[i] = pred_pose_x;
      pred_pose_y_vect[i] = pred_pose_y;

      float x_vel = cos(pred_pose_yaw)*linear_vel_teleop_;
      float y_vel = sin(pred_pose_yaw)*linear_vel_teleop_;
      float w_vel = angular_vel_teleop_;
      pred_pose_x += dt * x_vel;
      pred_pose_y += dt * y_vel;
      pred_pose_yaw += dt * w_vel;

      // Compute the minimum euclidean distance to the closest obstacle
      for (unsigned int j = 0; j < size; j += 1) {
        min_dist_i = std::sqrt(std::pow((x_obst[j]-pred_pose_x),2)+std::pow((y_obst[j]-pred_pose_y),2));
        if (min_dist > min_dist_i) {
          min_dist = min_dist_i;
          robot_colision_side_x = x[j];  // Closest point coordinates of the obstacle w.r.t. robot frame
          robot_colision_side_y = y[j];
        }
      }

      // Stop path iteration in case collision is detected
      if (min_dist < robot_radius_){
        collision = 1;
        t_impact = i;
        break;
      }
    }

    // Publishing predicted path to plot in RVIZ
    publish_marker(pred_pose_x_vect, pred_pose_y_vect, n_viz, "odom", "path", 2, 0.132, green, msg->header.stamp);

    // Publishing or Erasing collision point in the predicted path to plot in RVIZ
    if (collision){
      std::vector<double> robot_collision_x(1, robot_colision_side_x);
      std::vector<double> robot_collision_y(1, robot_colision_side_y);
      publish_marker(robot_collision_x, robot_collision_y, 1, "base_scan", "collision", 3, 0.132, red, msg->header.stamp);
    }
    else {
      std::vector<double> robot_collision_x(1, 0.0);
      std::vector<double> robot_collision_y(1, 0.0);
      publish_marker(robot_collision_x, robot_collision_y, 1, "base_scan", "collision", 3, 0.131, blue, msg->header.stamp);
    }

    // Compute distance to collision point
    dist_act_col = std::sqrt(std::pow((robot_colision_side_x),2)+std::pow((robot_colision_side_y),2));
    if (linear_vel_teleop_ < 0){
      dist_act_col -= 0.1;  // Coeficient used to prevent colision while going backwards (observed necessary in the simulation)
    }

    // Compute max linear velocity
    v_max_ = ( dist_act_col - robot_radius_ ) / t_safe_;

    // END TODO 1
}

void SafetyNode::publish_marker(const std::vector<double>& x,
                    const std::vector<double>& y,
                    const int&                 subsampling,
                    const std::string&         frame_id,
                    const std::string&         ns,
                    const int&                 id,
                    const double&              z,
                    const std_msgs::ColorRGBA& color,
                    const ros::Time&           t)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id    = frame_id;
    marker.header.stamp       = t;
    marker.ns                 = ns;
    marker.id                 = id;
    marker.type               = visualization_msgs::Marker::POINTS;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = 0;
    marker.pose.position.y    = 0;
    marker.pose.position.z    = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color              = color;
    marker.lifetime           = ros::Duration(0.0f);

    // Markers to plot obstacles
    if (id == 1) {
      marker.scale.x            = 0.1;
      marker.scale.y            = 0.1;
      marker.scale.z            = 0.1;
      marker.color.a = 1;
    }

    // Markers to plot robot predicted path
    else if (id == 2) {
      marker.scale.x            = 0.02;
      marker.scale.y            = 0.02;
      marker.scale.z            = 0.02;
      marker.color.a = 1;
    }

    // Markers to plot collision point
    else if (z == 0.132) {
      marker.scale.x            = 0.15;
      marker.scale.y            = 0.15;
      marker.scale.z            = 0.15;
      marker.color.a = 0.7;
    }
    // Markers to erase collition point while not colliding
    else if (z == 0.131){
      marker.scale.x            = 0.001;
      marker.scale.y            = 0.001;
      marker.scale.z            = 0.001;
      marker.color.a = 0.7;
    }

    geometry_msgs::Point point;
    for (unsigned int i = 0; i < x.size(); i += subsampling)
    {
        if (x[i] == x[i] and y[i] == y[i] and abs(x[i]) < 1e3 and abs(y[i]) < 1e3)
        {
            point.x = x[i];
            point.y = y[i];
            point.z = z;
            marker.points.push_back(point);
        }
    }
    // ROS_INFO("Publishing points of %s: %lu", ns.c_str(), marker.points.size());
    points_pub_.publish(marker);
}

void SafetyNode::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Save teleop variables from /cmd_vel_teleop topic
    linear_vel_teleop_ = msg->linear.x;
    angular_vel_teleop_ = msg->angular.z;

    geometry_msgs::Twist cmdvel_safe_msg = *msg; // copy of the received message

    // TODO 2: modify cmdvel_safe_msg reducing its speed if necessary

    // Computation of robot velocity according to situation, saturating velocities if necessary
    if (abs(robot_colision_side_y) < robot_radius_) {
      if (std::abs(cmdvel_safe_msg.linear.x) > std::abs(v_max_)) {
        if (cmdvel_safe_msg.linear.x >= 0) {
          if (robot_colision_side_x < 0) {
            cmdvel_safe_msg.linear.x = linear_vel_teleop_;
            cmdvel_safe_msg.angular.z = angular_vel_teleop_;
          }
          else{
            (dist_act_col < (robot_radius_) ? cmdvel_safe_msg.linear.x = 0 : cmdvel_safe_msg.linear.x = std::abs(v_max_));
            (linear_vel_teleop_ == 0 ? cmdvel_safe_msg.angular.z = angular_vel_teleop_ : cmdvel_safe_msg.angular.z = (std::abs(v_max_)/linear_vel_teleop_)*angular_vel_teleop_);
          }
        }
        else{
          if (robot_colision_side_x > 0) {
            cmdvel_safe_msg.linear.x = linear_vel_teleop_;
            cmdvel_safe_msg.angular.z = angular_vel_teleop_;
          }
          else{
            (dist_act_col < (robot_radius_) ? cmdvel_safe_msg.linear.x = 0 : cmdvel_safe_msg.linear.x = -std::abs(v_max_));
            (linear_vel_teleop_ == 0 ? cmdvel_safe_msg.angular.z = angular_vel_teleop_ : cmdvel_safe_msg.angular.z = -(std::abs(v_max_)/linear_vel_teleop_)*angular_vel_teleop_);
          }
        }
      }
    }

    // Send a message to rosout with the details.
     ROS_INFO_STREAM("Sending linear velocity command:"
       << " v_linear=" << cmdvel_safe_msg.linear.x
       << " v_angular=" << cmdvel_safe_msg.angular.z
       << " current_min_dist=" << dist_act_col);

    // END TODO 2

    // Publish the safe velocity command
    cmdvel_safe_pub_.publish(cmdvel_safe_msg);
}

void SafetyNode::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    nav_msgs::Odometry robot_pose = *odom_msg; // copy of the received message

    // Save pose variables
    pose_x   = odom_msg->pose.pose.position.x;
    pose_y   = odom_msg->pose.pose.position.y;
    pose_yaw = tf::getYaw(odom_msg->pose.pose.orientation);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh("~");
    SafetyNode      node_prudent(nh);
    ros::Rate       loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
