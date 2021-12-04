#ifndef COLUMN_IDENTYFY_H
#define COLUMN_IDENTYFY_H
#define RELEASE_COLUMN

#include <iostream>
#include <vector>
#include <pcl/search/impl/search.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <cmath>
#include <mutex>
#include <Eigen/Core>
#include <boost/thread.hpp>
#include <omp.h>
#include <unistd.h>
#include "livox.h"
#include "serialapp.h"
#include "MVS.h"
#include "slide_window.h"

namespace hitcrt {
    class column_identify {
    public:
        void run();

        void stop();

        column_identify();

        ~column_identify();

    private:
        // 用于对桶排序
        class col_color {
        public:
            col_color(cv::Point2f p, uchar color) : point(std::move(p)), color(color) {}

            cv::Point2f point;
            uchar color;

            bool operator<(const col_color &m) const {
                return point.x < m.point.x;
            }
        };

        enum COLOR_MODE {
            BLUE,
            RED
        };
        // 分别对应2号桶、1/3号桶、4/5号桶
        enum COLUMN_CLASS {
            HIGH,
            LOW,
            STATIC
        };
        bool stop_all, start_grab, start_serial;
        float camera_factor, camera_cx, camera_cy, camera_fx, \
            camera_fy, h_l, h_h, h_s, height, col_height, \
            lidar2camx, lidar2camy, lidar2camz, startyaw, \
            startx, starty, lidar_x, lidar_y, K;
        int CLOUD_SIZE, MIN_CLUSTER_SIZE, ACCEPT_SIZE, EXPOSURE;
        double B_low_H, B_low_S, B_low_V, B_high_H, B_high_S, \
            B_high_V, R_low_H, R_low_S, R_low_V, R_high_H, \
            R_high_S, R_high_V, CLUSTER_TOLERANCE, ACCEPT_R_L, ACCEPT_R_H, RNCDT;
        // 这个数组暂时还没有加入send_col中，预期是在当前的数据的最后加入5个数用于判断桶的动静
        // -1表示未知，0表示不动，1表示动
        bool col_move[5];
        std::vector<float> T, t, cal_T;
        std::vector<cv::Point2f> col_r;
        std::vector<cv::Point2f> col_b;
        std::vector<std::vector<col_color>> col_couples;
        boost::thread thread_cal_;
        boost::thread thread_grab_;
        boost::thread thread_serial_;
        boost::mutex serial_mu;
        boost::mutex grab_mu;
        LdsLidar &read_lidar = LdsLidar::GetInstance();
        cv::Mat grab_rgb;
        Eigen::Matrix3f trans_mat;
        pcl::ModelCoefficients::Ptr coefficients;
        hitcrt::SerialApp serial;
        hitcrt::SerialApp::RECEIVE_FLAG flag;
        hitcrt::slideWindow windows[5];
        ofstream outfile, outfile1;

        void column_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const COLUMN_CLASS choice);

        void send_col(COLOR_MODE mode);

        void thread_cal();

        void thread_grab();

        void thread_serial();

        static bool is_hidden(const cv::Point2f &A,const cv::Point2f &B);

        int detect_color(const cv::Point3f &input);

        cv::Point2f lidar2tr(const cv::Point2f &input) const;

        cv::Point2f lidar2world(cv::Point2f input, std::vector<float> m);

        cv::Point2f tr2world(cv::Point2f input, std::vector<float> m);

        cv::Point2f world2tr(cv::Point2f input, std::vector<float> m);

        static bool near(const cv::Point2f &A, const cv::Point2f &B, float distance);

        static std::string float2str(const std::vector<float> &input);
    };
}

#endif
