#ifndef LIVOX_H_
#define LIVOX_H_

#include <vector>
#include <string>
#include "livox_def.h"
#include "livox_sdk.h"
//#include <boost/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <memory>

typedef enum {
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
} LidarConnectState;

typedef enum {
    kConfigFan = 1,
    kConfigReturnMode = 2,
    kConfigCoordinate = 4,
    kConfigImuRate = 8
} LidarConfigCodeBit;

typedef enum {
    kCoordinateCartesian = 0,
    kCoordinateSpherical
} CoordinateType;

typedef struct {
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
} UserConfig;

typedef struct {
    uint8_t handle;
    LidarConnectState connect_state;
    DeviceInfo info;
    UserConfig config;
} LidarDevice;
/**
 * LiDAR data source, data from dependent lidar.
 */
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

class LdsLidar {
public:

    static LdsLidar &GetInstance() {
        static LdsLidar lds_lidar;
        return lds_lidar;
    }

    int InitLdsLidar(std::vector<std::string> &broadcast_code_strs,int max_size);

    int DeInitLdsLidar();

    void getcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    void clearcloud();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
    //static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    //std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud1;
    int max_cloud_size;
    boost::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    LdsLidar();

    LdsLidar(const LdsLidar &) = default;

    ~LdsLidar();

    LdsLidar &operator=(const LdsLidar &) = delete;

    static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, \
                             uint32_t data_num, void *client_data);

    static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);

    static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);

    static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);

    static void StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);

    static void DeviceInformationCb(livox_status status, uint8_t handle, \
                                  DeviceInformationResponse *ack, void *clent_data);

    static void LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message);

    static void ControlFanCb(livox_status status, uint8_t handle, \
                           uint8_t response, void *clent_data);

    static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                        uint8_t response, void *clent_data);

    static void SetCoordinateCb(livox_status status, uint8_t handle, \
                              uint8_t response, void *clent_data);

    static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                        uint8_t response, void *clent_data);

    int AddBroadcastCodeToWhitelist(const char *broadcast_code);

    void AddLocalBroadcastCode();

    bool FindInWhitelist(const char *broadcast_code);

    void EnableAutoConnectMode() { auto_connect_mode_ = true; }

    void DisableAutoConnectMode() { auto_connect_mode_ = false; }

    bool IsAutoConnectMode() const { return auto_connect_mode_; }

    bool auto_connect_mode_;
    uint32_t whitelist_count_;
    volatile bool is_initialized_;
    char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

    uint32_t lidar_count_;
    LidarDevice lidars_[kMaxLidarCount];
    uint32_t data_recveive_count_[kMaxLidarCount];
};

#endif
