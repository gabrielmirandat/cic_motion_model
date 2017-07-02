#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class Node
{
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber dsr_sub_, laser_sub_, odom_sub_;

    geometry_msgs::Twist command_vel_, desired_vel_;
    nav_msgs::Odometry pose_msg_;

    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);

public:
    Node(ros::NodeHandle nh);
    ~Node();

    void spin();
};

#endif // NODE_H
