#include <cstdio>
#include <thread>
#include "livox.h"
#include <pcl/io/pcd_io.h>

#define RECORD
#define SIZE 48000
/** Cmdline input broadcast code */
static std::vector<std::string> broadcast_code;

int main(int argc, const char *argv[]) {
    broadcast_code.emplace_back("0TFDFG700601861");
    broadcast_code.emplace_back("3WEDH7600110891");
    LdsLidar &read_lidar = LdsLidar::GetInstance();
    int ret = read_lidar.InitLdsLidar(broadcast_code, SIZE);
    if (!ret) {
        printf("Init lds lidar success!\n");
    } else {
        printf("Init lds lidar fail!\n");
    }
    printf("Start discovering device.\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr getcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::CloudViewer viewer("viewer");
    int pics = 0;
    while (!viewer.wasStopped()) {
        read_lidar.getcloud(getcloud);
        viewer.showCloud(getcloud);
#ifdef RECORD
        getcloud->height = 1;
        getcloud->width = getcloud->size();
        char path[50];
        pics++;
        sprintf(path, "../out/%d-%d.pcd", SIZE, pics);
        pcl::io::savePCDFileASCII(path, *getcloud);
        usleep(200000);
#endif
    }
    read_lidar.DeInitLdsLidar();
    printf("Livox lidar demo end!\n");

}
