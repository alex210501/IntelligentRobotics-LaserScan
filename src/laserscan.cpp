#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void messageCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    ROS_INFO("I heard something %f - %f", scan_in->angle_min, scan_in->angle_max);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laserscan");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scan", 1000, messageCallback);

    while (ros::ok()) ros::spin();

    return 0;
}