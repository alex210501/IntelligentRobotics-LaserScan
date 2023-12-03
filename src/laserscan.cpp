#include "lidar.h"


int main(int argc, char **argv) {
    Lidar lidar(argc, argv);

    lidar.run();

    return 0;
}