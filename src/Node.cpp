#include "Node.h"

Node::Node(ros::NodeHandle nh)
: first_time_(true), nh_(nh)
{
    odom_sub_ = nh_.subscribe("pose", 10, &Node::odomCallback, this);
}

void Node::odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    pose_msg_ = *pose_msg;
}


void Node::spin()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        // atualiza callbacks
        ros::spinOnce();

        // algorithm();
        // double x = pose_msg_.pose.pose.position.x;
        // double y = pose_msg_.pose.pose.position.y;
        // double theta = tf::getYaw(pose_msg_.pose.pose.orientation);
        // theta = RAD2DEGREE(theta);

        if(first_time_)
        {
            mm_.setOldControl(pose_msg_.pose.pose.position.x,
                              pose_msg_.pose.pose.position.y,
                              tf::getYaw(pose_msg_.pose.pose.orientation));
            first_time_ = false;
        }
        else
        {
            mm_.updateControl(pose_msg_.pose.pose.position.x,
                              pose_msg_.pose.pose.position.y,
                              tf::getYaw(pose_msg_.pose.pose.orientation));
        }

        mm_.run();
        loop_rate.sleep();
    }
}
