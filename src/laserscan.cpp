#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

ros::Publisher pub;

typedef sensor_msgs::PointCloud PointCloud;
typedef geometry_msgs::Point32 Point32;
typedef sensor_msgs::ChannelFloat32 ChannelFloat32;

void messageCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    int numberPoints = (scan_in->angle_max - scan_in->angle_min) / scan_in->angle_increment;
    PointCloud scan_out;
    ChannelFloat32 channel;
    ROS_INFO("I heard something %f - %f", scan_in->angle_min, scan_in->angle_max);

    scan_out.header.frame_id = scan_in->header.frame_id;

    for (int i = 0; i < numberPoints; i++) {
        double angle = i * scan_in->angle_increment;
        double range = scan_in->ranges[i];
        Point32 point;

        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0;
        channel.values.push_back(10);

        scan_out.points.push_back(point);
    }

    scan_out.channels.push_back(channel);

    pub.publish(scan_out);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laserscan");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scan", 1000, messageCallback);
    pub = n.advertise<PointCloud>("/out", 1000);

    while (ros::ok()) ros::spin();

    return 0;
}