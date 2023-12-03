#ifndef __CLUSTER_H__
#define __CLUSTER_H__

#include "defines.h"


#define CLUSTER_RADIUS (0.4)  // m

double getDistance(Point32 a, Point32 b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

class Cluster {
    std::vector<Point32> points;

public:
    static void buildFromPoint(std::vector<Cluster>& clusters, std::vector<Point32>& points) {
        for (int i = 0; i < points.size(); i++) {
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
    }

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

#endif /* __CLUSTER_H__ */