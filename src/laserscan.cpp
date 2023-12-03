#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"


#define CLUSTER_RADIUS (0.4)  // m

ros::Publisher pubCoords, pubPersons;

typedef sensor_msgs::PointCloud PointCloud;
typedef geometry_msgs::Point32 Point32;
typedef sensor_msgs::ChannelFloat32 ChannelFloat32;


class Cluster {
    std::vector<Point32> points;

public:
    void addPoint(Point32 p) { points.push_back(p); }

    Point32 getLastPoint() { 
        if (points.empty()) return {};

        return points.back();
    }

    Point32 getAverage() {
        if (points.empty()) return {};

        double x = 0.0, y = 0.0;

        for (auto p: points) {
            x += p.x;
            y += p.y;
        }

        Point32 p;

        p.x = x / points.size();
        p.y = y / points.size();

        return p;
    }
};

double getDistance(Point32 a, Point32 b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void messageCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    int numberPoints = (scan_in->angle_max - scan_in->angle_min) / scan_in->angle_increment;
    PointCloud coords, persons;
    ChannelFloat32 coordsChannel, personsChannel;
    std::vector<Point32> points;
    std::vector<Cluster> clusters;

    coords.header.frame_id = scan_in->header.frame_id;
    persons.header.frame_id = scan_in->header.frame_id;

    // Convert polar to x, y coordinates
    for (int i = 0; i < numberPoints; i++) {
        double angle = i * scan_in->angle_increment;
        double range = scan_in->ranges[i];

        // Add only valid ranges
        if (isinf(range)) continue;
        
        Point32 p;
        
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        p.z = 0;

        points.push_back(p);
        coordsChannel.values.push_back(10);
        coords.points.push_back(p);
    }

    for (int i = 0; i < points.size() - 1; i++) {
        for (int j = 0; j < clusters.size() + 1; j++) {
            if (j == clusters.size()) {
                clusters.push_back({});
                clusters.back().addPoint(points[i]);
                break;
            }

            Point32 lastPoint = clusters[j].getLastPoint();

            if (getDistance(points[i], lastPoint) < CLUSTER_RADIUS) {
                clusters[j].addPoint(points[i]);
                break;
            }
        }
    }

    // Get position of each person by computing the average of each cluster
    for (auto c: clusters) {
        persons.points.push_back(c.getAverage());
        personsChannel.values.push_back(30);
    }

    ROS_INFO("%d", clusters.size());
    coords.channels.push_back(coordsChannel);
    persons.channels.push_back(personsChannel);

    pubCoords.publish(coords);
    pubPersons.publish(persons);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laserscan");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scan", 1000, messageCallback);
    pubCoords = n.advertise<PointCloud>("/coords", 1000);
    pubPersons = n.advertise<PointCloud>("/persons", 1000);

    while (ros::ok()) ros::spin();

    return 0;
}