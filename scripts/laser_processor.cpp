#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_datatypes.h>

class LaserProcessor
{
  public:
    LaserProcessor()
    {
        this->laser_sub  = n.subscribe("scan", 1, &LaserProcessor::laser_callback, this);
        this->odom_sub   = n.subscribe("odom", 1, &LaserProcessor::odom_callback, this);
        this->points_pub = n.advertise<visualization_msgs::Marker>("points", 1);

        this->red.r   = 1.0;
        this->red.a   = 0.5;
        this->green.g = 1.0;
        this->green.a = 0.5;
        this->blue.b  = 1.0;
        this->blue.a  = 0.5;

        // transform from odom to platform (automatically updated on odom_callback)
        this->pose_x   = 0.0;
        this->pose_y   = 0.0;
        this->pose_yaw = 0.0;

        // transform from platform to sensor
        // TODO 2a START
        // this->sensor_x   =
        // this->sensor_y   =
        // this->sensor_z   =
        // this->sensor_yaw =
        // TODO 2a END
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
    {
        int size = laser_msg->ranges.size();
        ROS_INFO("Received scan ranges size=%d", size);
        std::vector<double> x(size, 0.0);
        std::vector<double> y(size, 0.0);

        // print & visualize one of each n points
        int n_viz   = 10;        // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS
        int n_print = size / 4;  // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS

        // ======= C++ TIPS: Some examples of using std::vector =======

        // //Resize a vector
        // std::vector<int> v;
        // int my_size=2;
        // ROS_INFO("v.size()=%lu", v.size());
        // v.resize(my_size);
        // ROS_INFO("v.size()=%lu", v.size());
        // ROS_INFO("--");

        // //Add an element at the end
        // std::vector<double> vv;
        // double my_number=1.23;
        // double other_number=4.56;
        // ROS_INFO("vv.size()=%lu", vv.size());
        // vv.push_back(my_number);
        // ROS_INFO("vv.size()=%lu", vv.size());
        // vv.push_back(other_number);
        // ROS_INFO("vv.size()=%lu", vv.size());
        // ROS_INFO("--");

        // //Init vector with size and value
        // int other_size=3;
        // double value=7.89;
        // std::vector<double> vvv(other_size,value);
        // //Loop through a vector
        // for(unsigned int i=0; i<vvv.size(); i++)
        // {
        //  ROS_INFO("vvv[%d]=%f",i, vvv[i]);
        // }
        // ROS_INFO("--");

        // //

        // ======== TRANSFORMATION 1 ========
        // scan from polar to cartesian (local coordinates)

        // TODO 1 START
        //
        //
        //
        // TODO 1 END

        // OUTPUT1
        ROS_INFO("Cartesian (scan frame):");
        for (unsigned int i = 0; i < x.size(); i += n_print) ROS_INFO("for index=%d, x,y=%f,%f", i, x[i], y[i]);

        this->publish_marker(x, y, n_viz, "base_scan", "cartesian", 1, 0.0, red, laser_msg->header.stamp);

        // ======== TRANSFORMATION 2 ========
        // scan points in global coordinates

        // TODO 2b & 2c START
        //
        //
        //
        // TODO 2b & 2c END

        // OUTPUT2
        ROS_INFO("World frame:");
        for (unsigned int i = 0; i < x.size(); i += n_print) ROS_INFO("for index=%d, x,y=%f,%f", i, x[i], y[i]);

        this->publish_marker(x, y, n_viz, "odom", "world", 2, this->sensor_z, blue, laser_msg->header.stamp);

        ROS_INFO("---");
    }

    void publish_marker(const std::vector<double>& x,
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
        marker.scale.x            = 0.01;
        marker.scale.y            = 0.01;
        marker.scale.z            = 0.01;
        marker.color              = color;
        marker.lifetime           = ros::Duration(0.0f);

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
        ROS_INFO("Publishing points of %s: %lu", ns.c_str(), marker.points.size());
        this->points_pub.publish(marker);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        this->pose_x   = odom_msg->pose.pose.position.x;
        this->pose_y   = odom_msg->pose.pose.position.y;
        this->pose_yaw = tf::getYaw(odom_msg->pose.pose.orientation);
    }

  private:
    ros::NodeHandle     n;
    ros::Subscriber     laser_sub;
    ros::Subscriber     odom_sub;
    ros::Publisher      points_pub;
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    std_msgs::ColorRGBA blue;
    double              pose_x, pose_y, pose_yaw;
    double              sensor_x, sensor_y, sensor_z, sensor_yaw;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_processor");
    LaserProcessor my_laser_processor;
    ros::Rate      loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}