#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_datatypes.h>

class OdomNew
{
  private:

    ros::NodeHandle n_; 
    ros::Subscriber odom_sub_, joint_state_sub_;
    ros::Publisher  odom_pub_;

    bool odom_initialized_, received_joint_state_;
    sensor_msgs::JointState joint_state_msg_;
    nav_msgs::Odometry odom_msg_;
    ros::Time last_time_;
    
    double x_,y_,th_;
    double radius_, wheel_distance_;

  public:

    OdomNew()
    {
      joint_state_sub_  = n_.subscribe("joint_states", 1, &OdomNew::joint_state_callback, this);
      odom_sub_         = n_.subscribe("odom", 1, &OdomNew::odom_callback, this);
      
      odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom_new", 1);

      received_joint_state_ = false;
      odom_initialized_     = false;
      
      x_  = 0.0;
      y_  = 0.0;
      th_ = 0.0;
      
      radius_         = 0.033;
      wheel_distance_ = 0.288;
      last_time_      = ros::Time::now();
    }

    void compute_and_publish_odom()
    {
      // skip if not received joint states or not initialized
      if(not received_joint_state_ or not odom_initialized_)
        return;

      //ROS_INFO("compute_and_publish_odom");
      double right_angular_vel = joint_state_msg_.velocity[0];
      double left_angular_vel  = joint_state_msg_.velocity[1];

      ros::Time time = joint_state_msg_.header.stamp;
      //ROS_INFO("Received left_angular_vel: %f (rad/s)\t right_angular_vel: %f (rad/s)", left_angular_vel, right_angular_vel);
      double dt = (time - last_time_).toSec();
      //ROS_INFO("dt: %fs", dt);
      double vx=0.0;
      double vth=0.0;
      
      //////// TODO //////
      //vx  = 
      //vth = 
      //x_  = 
      //y_  = 
      //th_ = 
      //////// TODO //////
      
      odom_msg_.header.stamp = time;
      odom_msg_.header.frame_id = "odom";
      odom_msg_.pose.pose.position.x = x_;
      odom_msg_.pose.pose.position.y = y_;
      odom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
      
      odom_msg_.child_frame_id = "base_footprint";
      odom_msg_.twist.twist.linear.x = vx;
      odom_msg_.twist.twist.angular.z = vth;
      
      odom_pub_.publish(odom_msg_);
      last_time_=time;
    }

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      if(msg->position.size()>=2 and msg->velocity.size()>=2)
      {
        ROS_ERROR("Wrong input joint_state size");
        return;
      }

      // Turtlebot3 does not fill the fields 'joint_state_msg_.velocity', we compute them using position increments
      if (joint_state_msg_.position.empty()) // First time just copy the message
      {
        joint_state_msg_ = *msg;
      }
      else // Second time we can compute (and fill) velocity
      {
        double dt = (msg->header.stamp - joint_state_msg_.header.stamp).toSec();
      
        joint_state_msg_.velocity.resize(2);
        joint_state_msg_.velocity[0] = (msg->position[0]-joint_state_msg_.position[0]) / dt;
        joint_state_msg_.velocity[1] = (msg->position[1]-joint_state_msg_.position[1]) / dt;
        joint_state_msg_.position = msg->position;
        joint_state_msg_.header = msg->header;

        received_joint_state_=true;
      }
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      if(received_joint_state_ and not odom_initialized_)
      {
        ROS_INFO("Received input odom, initializing new odom");
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        th_ = tf::getYaw(msg->pose.pose.orientation);
        last_time_ = msg->header.stamp;

        ROS_INFO("New odom initialized from input odom x_,y_,th_=%f,%f,%f", x_,y_,th_);
        odom_initialized_=true;
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_new");
  OdomNew my_odom_new;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_odom_new.compute_and_publish_odom();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
