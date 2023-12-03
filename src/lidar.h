#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "defines.h"

#define SCAN_TOPIC     ("/scan")
#define COORDS_TOPIC   ("/coords")
#define PERSONS_TOPIC  ("/persons")


class Lidar {
    ros::NodeHandle n;
    ros::Publisher pubCoords, pubPersons;
    ros::Subscriber sub;

    void polarToCoordinate(const LaserScanConstPtr&, PointCloud&);

public:
    Lidar(int, char **);
    void run();
    void messageCallback(const LaserScanConstPtr&);
};

#endif  /* __LIDAR_H__ */