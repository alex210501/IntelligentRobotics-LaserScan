#include "cluster.h"
#include "lidar.h"

Lidar::Lidar(int argc, char **argv) {
    ros::init(argc, argv, "laserscan");

    sub = n.subscribe(SCAN_TOPIC, 1000, &Lidar::messageCallback, this);
    pubCoords = n.advertise<PointCloud>(COORDS_TOPIC, 1000);
    pubPersons = n.advertise<PointCloud>(PERSONS_TOPIC, 1000);
}

void Lidar::run() {
     while (ros::ok()) ros::spin();
}

void Lidar::messageCallback(const LaserScanConstPtr& scan_in) {
    PointCloud coords, persons;
    ChannelFloat32 personsChannel;
    std::vector<Cluster> clusters;

    coords.header.frame_id = scan_in->header.frame_id;
    persons.header.frame_id = scan_in->header.frame_id;

    polarToCoordinate(scan_in, coords);
    Cluster::buildFromPoint(clusters, coords.points);

    // Get position of each person by computing the average of each cluster
    for (auto c: clusters) {
        persons.points.push_back(c.getAverage());
        personsChannel.values.push_back(30);
    }

    persons.channels.push_back(personsChannel);

    pubCoords.publish(coords);
    pubPersons.publish(persons);
}

void Lidar::polarToCoordinate(const LaserScanConstPtr& in, PointCloud& pc) {
    ChannelFloat32 channel;
    int numberPoints = (in->angle_max - in->angle_min) / in->angle_increment;

    for (int i = 0; i < numberPoints; i++) {
        double angle = i * in->angle_increment;
        double range = in->ranges[i];

        // Add only valid ranges
        if (isinf(range)) continue;
        
        Point32 p;
        
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        p.z = 0;

        channel.values.push_back(10);
        pc.points.push_back(p);
    }

    pc.channels.push_back(channel);
} 