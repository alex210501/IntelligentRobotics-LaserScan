#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "defines.h"

class Lidar {
    ros::NodeHandle n;
    ros::Publisher pubCoords, pubPersons;
    ros::Subscriber sub;

    void polarToCoordinate(const LaserScanConstPtr&, PointCloud&);

public:
    Lidar();
    void run();
    void messageCallback(const LaserScanConstPtr&);
};

#endif  /* __LIDAR_H__ */