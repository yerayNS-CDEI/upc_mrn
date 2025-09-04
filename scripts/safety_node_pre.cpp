#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// Variable definitions
float min_dist = 10e9;
float min_dist_act = 1;

// Class definition
class SafetyNode
{
  private:
    ros::Publisher  cmdvel_safe_pub_;
    ros::Subscriber cmdvel_sub_, laser_sub_;
    double          t_safe_, v_max_, robot_radius_;
    double          robot_colision_side_x, robot_colision_side_y;
    double          w_max_, linear_vel_teleop_, angular_vel_teleop_;

  public:
    SafetyNode(ros::NodeHandle& nh);

  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
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
    cmdvel_safe_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void SafetyNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // TODO 1: compute the maximum velocity allowed
    int size = msg->ranges.size();

    // Transform from polar to cartesian
    std::vector<double> x(size, 0.0);
    std::vector<double> y(size, 0.0);
    for (unsigned int i = 0; i < x.size(); i += 1)
      x[i] = msg->ranges[i] * cos(i*msg->angle_increment) - 0.064;
    for (unsigned int i = 0; i < y.size(); i += 1)
      y[i] = msg->ranges[i] * sin(i*msg->angle_increment);

    // Compute the minimum euclidean distance to the closest obstacle
    min_dist = 10e9;
    min_dist_act = 1;
    for (unsigned int i = 0; i < size; i += 1) {
      min_dist_act = std::sqrt(std::pow(x[i],2)+std::pow(y[i],2));
      if (min_dist > min_dist_act) {
        min_dist = min_dist_act;
        robot_colision_side_x = x[i];
        robot_colision_side_y = y[i];
      }
    }

    // Compute max linear velocity
    v_max_ = ( min_dist - robot_radius_ ) / t_safe_;

    // END TODO 1
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
            (min_dist < (robot_radius_) ? cmdvel_safe_msg.linear.x = 0 : cmdvel_safe_msg.linear.x = std::abs(v_max_));
            (linear_vel_teleop_ == 0 ? cmdvel_safe_msg.angular.z = angular_vel_teleop_ : cmdvel_safe_msg.angular.z = (std::abs(v_max_)/linear_vel_teleop_)*angular_vel_teleop_);
          }
        }
        else{
          if (robot_colision_side_x > 0) {
            cmdvel_safe_msg.linear.x = linear_vel_teleop_;
            cmdvel_safe_msg.angular.z = angular_vel_teleop_;
          }
          else{
            (min_dist < (robot_radius_) ? cmdvel_safe_msg.linear.x = 0 : cmdvel_safe_msg.linear.x = -std::abs(v_max_));
            (linear_vel_teleop_ == 0 ? cmdvel_safe_msg.angular.z = angular_vel_teleop_ : cmdvel_safe_msg.angular.z = -(std::abs(v_max_)/linear_vel_teleop_)*angular_vel_teleop_);
          }
        }
      }
    }


    // Send a message to rosout with the details.
     ROS_INFO_STREAM("Sending linear velocity command:"
       << " v_linear=" << cmdvel_safe_msg.linear.x
       << " v_angular=" << cmdvel_safe_msg.angular.z
       << " min_dist=" << min_dist);

    // END TODO 2

    // publish the safe velocity command
    cmdvel_safe_pub_.publish(cmdvel_safe_msg);
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
