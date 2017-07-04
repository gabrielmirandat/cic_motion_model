#include "Node.h"

Node::Node(ros::NodeHandle nh)
: first_time_(true), nh_(nh), pnh_("~")
{
    double alpha1, alpha2, alpha3, alpha4;
    odom_sub_ = nh_.subscribe("pose", 10, &Node::odomCallback, this);
    pnh_.param("alpha1", alpha1, 0.01);
    pnh_.param("alpha2", alpha2, 0.01);
    pnh_.param("alpha3", alpha3, 0.01);
    pnh_.param("alpha4", alpha4, 0.01);

    mm_.setAlpha(alpha1, alpha2, alpha3, alpha4);
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
            mm_.setOldControl(0,
                              0,
                              0);
             mm_.updateControl(pose_msg_.pose.pose.position.x,
                              pose_msg_.pose.pose.position.y,
                              tf::getYaw(pose_msg_.pose.pose.orientation));
            first_time_ = false;
            continue;
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
