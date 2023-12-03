#include "cluster.h"
#include "lidar.h"

#define COORDS_COLOR   (0x00ff00)
#define PERSONS_COLOR  (0xffa500)
#define SCAN_TOPIC     ("/scan")
#define COORDS_TOPIC   ("/coords")
#define PERSONS_TOPIC  ("/persons")

Lidar::Lidar() {
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
        int rgb = PERSONS_COLOR;
        
        persons.points.push_back(c.getAverage());
        personsChannel.values.push_back(*reinterpret_cast<float*>(&rgb));
    }

    personsChannel.name = "rgb";
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

        int rgb = COORDS_COLOR;        
        Point32 p;
        
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        p.z = 0;

        channel.values.push_back(*reinterpret_cast<float*>(&rgb));
        pc.points.push_back(p);
    }

    channel.name = "rgb";
    pc.channels.push_back(channel);
} 