#include "Node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cic_motion_model");
    ros::NodeHandle n;                  // node handle global
    Node node(n);
    node.spin();
}

