#include "Node.h"

Node::Node(ros::NodeHandle nh)
: nh_(nh)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    dsr_sub_ = nh_.subscribe("desired_vel", 10, &Node::dsrCallback, this);
    laser_sub_ = nh_.subscribe("hokuyo_scan", 10, &Node::laserCallback, this);
    odom_sub_ = nh_.subscribe("pose", 10, &Node::odomCallback, this);
}

void Node::spin()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        // atualiza callbacks
        ros::spinOnce();

        // algorithm();
        vel_pub_.publish(command_vel_);
        loop_rate.sleep();
    }

    command_vel_.linear.x = 0.0;
    command_vel_.angular.z = 0.0;
    vel_pub_.publish(command_vel_);
}
