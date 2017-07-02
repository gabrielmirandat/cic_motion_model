#include "Node.h"

Node::Node(ros::NodeHandle nh)
: nh_(nh)
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
        double x = pose_msg_.pose.pose.position.x;
        double y = pose_msg_.pose.pose.position.y;
        double theta = tf::getYaw(pose_msg_.pose.pose.orientation);
        theta = RAD2DEGREE(theta);

        std::cout << x << ";" << y << ";" << theta << std::endl;

        loop_rate.sleep();
    }
}
