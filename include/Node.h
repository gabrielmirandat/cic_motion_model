#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <MotionModel.h>

#define POSANGLE(a) (((a) > 0.0)? (a) : ((a) + 360.0))
#define RAD2DEGREE(a) (POSANGLE((a)*180.0/M_PI))

class Node
{
private:
    bool first_time_;
    MotionModel mm_;

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry pose_msg_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);

public:
    Node(ros::NodeHandle nh);

    void spin();
};

#endif // NODE_H
