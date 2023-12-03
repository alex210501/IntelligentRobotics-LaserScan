#include "lidar.h"


#define NODE_NAME ("laserscan")

int main(int argc, char **argv) {
    // Start node
    ros::init(argc, argv, NODE_NAME);

    Lidar lidar;
    lidar.run();

    return 0;
}