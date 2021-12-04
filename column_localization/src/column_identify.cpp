#include "column_identify.h"

using namespace hitcrt;

column_identify::column_identify() : col_couples(5), serial("auto", 115200) {
    time_t time_log = time(nullptr);
    char tmp[100];
    strftime(tmp, sizeof(tmp), "../log/%Y-%m-%d %H:%M:%S.dat", localtime(&time_log));
    outfile.open(tmp, ios::out | ios::trunc);
    outfile1.open("../log/paraments.txt", ios::app | ios::out);
    cv::FileStorage detectcfg("../cfg/detect.yml", cv::FileStorage::READ);
    detectcfg["height"] >> height;
    detectcfg["CLUSTER_TOLERANCE"] >> CLUSTER_TOLERANCE;
    detectcfg["MIN_CLUSTER_SIZE"] >> MIN_CLUSTER_SIZE;
    detectcfg["h_l"] >> h_l;
    detectcfg["h_h"] >> h_h;
    detectcfg["h_s"] >> h_s;
    detectcfg["ACCEPT_SIZE"] >> ACCEPT_SIZE;
    detectcfg["ACCEPT_R_L"] >> ACCEPT_R_L;
    detectcfg["ACCEPT_R_H"] >> ACCEPT_R_H;
    detectcfg["RNCDT"] >> RNCDT;
    detectcfg["CLOUD_SIZE"] >> CLOUD_SIZE;
    detectcfg["lidar_x"] >> lidar_x;
    detectcfg["lidar_y"] >> lidar_y;
    detectcfg["col_height"] >> col_height;
    detectcfg["startx"] >> startx;
    detectcfg["starty"] >> starty;
    detectcfg["startyaw"] >> startyaw;
    detectcfg["lidar2camx"] >> lidar2camx;
    detectcfg["lidar2camy"] >> lidar2camy;
    detectcfg["lidar2camy"] >> lidar2camz;
    detectcfg["camera_factor"] >> camera_factor;
    detectcfg["camera_cx"] >> camera_cx;
    detectcfg["camera_cy"] >> camera_cy;
    detectcfg["camera_fx"] >> camera_fx;
    detectcfg["camera_fy"] >> camera_fy;
    detectcfg["R_low_H"] >> R_low_H;
    detectcfg["R_low_S"] >> R_low_S;
    detectcfg["R_low_V"] >> R_low_V;
    detectcfg["R_high_H"] >> R_high_H;
    detectcfg["R_high_S"] >> R_high_S;
    detectcfg["R_high_V"] >> R_high_V;
    detectcfg["B_low_H"] >> B_low_H;
    detectcfg["B_low_S"] >> B_low_S;
    detectcfg["B_low_V"] >> B_low_V;
    detectcfg["B_high_H"] >> B_high_H;
    detectcfg["B_high_S"] >> B_high_S;
    detectcfg["B_high_V"] >> B_high_V;
    detectcfg["EXPOSURE"] >> EXPOSURE;
    detectcfg.release();
    trans_mat << camera_fx, 0, camera_cx, 0, camera_fy, camera_cy, 0, 0, camera_factor;
    std::vector<std::string> broadcast_code;
    broadcast_code.emplace_back("0TFDFG700601861");
    int ret = read_lidar.InitLdsLidar(broadcast_code, CLOUD_SIZE);
    if (!ret) {
        printf("Init lds lidar success!\n");
    } else {
        printf("Init lds lidar fail!\n");
    }
    t = {0, 0, 0, 0};
    start_grab = false;
    stop_all = false;
    start_serial = false;
}

void column_identify::run() {
    thread_grab_ = boost::thread([this] { thread_grab(); });
    thread_serial_ = boost::thread([this] { thread_serial(); });
    thread_cal_ = boost::thread([this] { thread_cal(); });
    std::cout << "----started----" << std::endl;
}

void column_identify::thread_grab() {
    MVS camera;
    camera.open(0, 1280, 1024);   //此函数根据需要可以更改
    camera.get_param("exposure"); //输出参数值
    camera.get_param("width");
    camera.get_param("height");
    camera.set_param("gamma", 0, 0.7); //设置参数值,自己看源码怎么改，p1为int型参数，p2为float型
    camera.set_param("exposure", 0, EXPOSURE);
    cv::Mat test;
    cv::Mat mask_temp;
    cv::Mat mask_b;
    cv::Mat mask_r;
#ifndef RELEASE_COLUMN
    cv::namedWindow("test");
    char name[40];
    static int q1 = 1;
    struct timeval tv
    {
    };
    gettimeofday(&tv, nullptr);
    long ago;
    ago = tv.tv_sec * 1000 + tv.tv_usec / 1000;
#endif
    do {
#ifndef RELEASE_COLUMN
        sprintf(name, "../p/%d.jpg", q1);
        q1++;
#endif
        if (!camera.grab(test)) {
            usleep(1000);
            continue;
        }
        cv::blur(test, test, cv::Size2d(7, 7));
        cv::cvtColor(test, test, cv::COLOR_BGR2HSV);
        cv::Mat show(test.rows, test.cols, CV_8UC1, 255);
        cv::Mat show_b(test.rows, test.cols, CV_8UC1, BLUE);
        cv::Mat show_r(test.rows, test.cols, CV_8UC1, RED);
        cv::inRange(test, cv::Scalar(B_low_H, B_low_S, B_low_V), cv::Scalar(B_high_H, B_high_S, B_high_V), mask_b);
        cv::inRange(test, cv::Scalar(0, R_low_S, R_low_V), cv::Scalar(R_low_H, R_high_S, R_high_V), mask_temp);
        mask_temp.copyTo(mask_r);
        cv::inRange(test, cv::Scalar(R_high_H, R_low_S, R_low_V), cv::Scalar(180, R_high_S, R_high_V), mask_temp);
        mask_temp.copyTo(mask_r, mask_temp);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
        cv::morphologyEx(mask_r, mask_r, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask_r, mask_r, cv::MORPH_OPEN, element, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask_b, mask_b, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask_b, mask_b, cv::MORPH_OPEN, element, cv::Point(-1, -1), 1);
        show_b.copyTo(show, mask_b);
        show_r.copyTo(show, mask_r);
        grab_mu.lock();
        this->grab_rgb = show.clone();
        grab_mu.unlock();
        start_grab = true;
#ifndef RELEASE_COLUMN
        //cv::imshow("intergreted", origin);
        cv::imshow("test", show);
        cv::waitKey(1);
        gettimeofday(&tv, nullptr);
        std::cout << "--------------"
                  << "grab time:" << tv.tv_sec * 1000 + tv.tv_usec / 1000 - ago << std::endl;
        ago = tv.tv_sec * 1000 + tv.tv_usec / 1000;
#endif
        //std::cerr << "grab image failed!" << std::endl;
    } while (!stop_all);
    camera.close();
}

void column_identify::thread_serial() {
    std::vector<float> data;
    if (serial.receive(flag, data)) {
        if (flag == SerialApp::RECEIVE_FLAG::PARAMENTS)
        {
            char tmp[50];
            time_t curr_time = time(nullptr);
            strftime(tmp, 50, "%m-%d %H:%M:%S", localtime(&curr_time));
            outfile1 << tmp;
            for (uint8_t i = 0; i < data.size(); ++i)
                outfile1 << data[i];
            outfile1 << std::endl;
        }
        serial_mu.lock();
        t.assign(data.begin(), data.end());
        serial_mu.unlock();
        start_serial = true;
    }
    while (!stop_all) {
        if (serial.receive(flag, data)) {
            if (flag == SerialApp::RECEIVE_FLAG::PARAMENTS)
            {
                char tmp[50];
                time_t curr_time = time(nullptr);
                strftime(tmp, 50, "%m-%d %H:%M:%S", localtime(&curr_time));
                outfile1 << tmp;
                for (uint8_t i = 0; i < data.size(); ++i)
                    outfile1 << data[i];
                outfile1 << std::endl;
            }
            serial_mu.lock();
            t.assign(data.begin(), data.end());
            serial_mu.unlock();
        }
    }
}

void column_identify::column_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const COLUMN_CLASS choice) {
    pcl::ProjectInliers<pcl::PointXYZ>::Ptr proj(new pcl::ProjectInliers<pcl::PointXYZ>);
    proj->setModelType(pcl::SACMODEL_PLANE);
    proj->setInputCloud(input_cloud);
    proj->setModelCoefficients(coefficients);
    proj->filter(*input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize(2000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);
    for (auto &cluster_index : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int index : cluster_index.indices) {
            cloud_cluster->push_back((*input_cloud)[index]);
        }
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients modelParas;
        seg.setOptimizeCoefficients(false);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_LMEDS);
        seg.setMaxIterations(50);
        seg.setNumberOfThreads(-1);
        seg.setDistanceThreshold(RNCDT);
        seg.setRadiusLimits(ACCEPT_R_L, ACCEPT_R_H);
        seg.setInputCloud(cloud_cluster);
        seg.segment(cluster_index, modelParas);
        if (modelParas.values.empty()) {
            continue;
        }
        float U = modelParas.values.at(0);
        float V = modelParas.values.at(1);
        float R = modelParas.values.at(2);
        if (cluster_index.indices.size() * sqrt(pow(U, 2) + pow(V, 2)) > ACCEPT_SIZE
            && R > ACCEPT_R_L && R < ACCEPT_R_H) {
            cv::Point2f column = {U, V};
            cv::Point2f col_w = lidar2world(column, T);
            switch (choice) {
                case COLUMN_CLASS::HIGH: {
                    if (near(col_w, {6000, 6000}, 1200)) {
                        col_color colColor(column, detect_color({U, V, h_h - height + col_height - 80}));
                        col_couples[1].push_back(colColor);
                    }
                    break;
                }
                case COLUMN_CLASS::LOW: {
                    if (near(col_w, {3500, 6000}, 1200)) {
                        col_color colColor(column, detect_color({U, V, h_l - height + col_height - 80}));
                        col_couples[0].push_back(colColor);
                    } else if (near(col_w, {8500, 6000}, 1200)) {
                        col_color colColor(column, detect_color({U, V, h_l - height + col_height - 80}));
                        col_couples[2].push_back(colColor);
                    }
                    break;
                }
                case COLUMN_CLASS::STATIC: {
                    if (near(col_w, {6000, 4000}, 1200)) {
                        col_color colColor(column, detect_color({U, V, h_s - height + col_height - 80}));
                        col_couples[3].push_back(colColor);
                    } else if (near(col_w, {6000, 8000}, 1200)) {
                        col_color colColor(column, detect_color({U, V, h_s - height + col_height - 80}));
                        col_couples[4].push_back(colColor);
                    }
                    break;
                }
            }
        }
    }
}

void column_identify::thread_cal() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // low - 矮桶
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l(new pcl::PointCloud<pcl::PointXYZ>);
    // high - 高桶
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_h(new pcl::PointCloud<pcl::PointXYZ>);
    // static - 定桶
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
    coefficients.reset(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
#ifndef RELEASE_COLUMN
    static pcl::visualization::CloudViewer viewer("viewer");
#endif
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    long ago;
    ago = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    tm *st = gmtime(&tv.tv_sec);
    outfile << "[" << st->tm_hour << "-" << st->tm_min << "-" << st->tm_sec << "-" << tv.tv_usec % 100000 << "]";
    // 以红场标定的实际桶的位置
    std::vector<cv::Point2f> world_match;
    if (flag == SerialApp::RECEIVE_FLAG::RED) {
        world_match = {{3505, 6010},
                       {6002, 6003},
                       {8488, 6013},
                       {6272, 4010},
                       {5722, 4012},
                       {6280, 8007},
                       {5726, 7995}};
        // world_match = {{3500, 6000},
        //                {6000, 6000},
        //                {8500, 6000},
        //                {6280, 4000},
        //                {5720, 4000},
        //                {6280, 8000},
        //                {5720, 8000}};
    } else if (flag == SerialApp::RECEIVE_FLAG::BLUE) {
        world_match = {{3512, 5987},
                       {5998, 5997},
                       {8495, 5990},
                       {5720, 3993},
                       {6274, 4005},
                       {5728, 7990},
                       {6278, 7988}};
    }
    while (!start_serial || !start_grab) {
        usleep(1000);
    }
    do {
        for (uint8_t i = 0; i < 5; ++i)
            col_couples[i].clear();
        col_r = {{0, 0},
                 {0, 0},
                 {0, 0},
                 {0, 0},
                 {0, 0}};
        col_b = {{0, 0},
                 {0, 0},
                 {0, 0},
                 {0, 0},
                 {0, 0}};
        // 3000 for wrong angle
        cal_T = {0, 0, 3000};
        read_lidar.getcloud(cloud);
        pcl::PassThrough<pcl::PointXYZ>::Ptr pass(new pcl::PassThrough<pcl::PointXYZ>);
        pass->setInputCloud(cloud);
        pass->setFilterFieldName("x");
        pass->setFilterLimits(1000, 12000);
        pass->setNegative(false);
        pass->filter(*cloud);
        if (cloud->size() < 100) {
            usleep(10000);
            continue;
        }
        pass->setFilterFieldName("z");
        pass->setFilterLimits(h_l - height, h_l - height + col_height);
        pass->filter(*cloud_l);
        pass->setFilterLimits(h_h - height, h_h - height + col_height);
        pass->filter(*cloud_h);
        pass->setFilterLimits(h_s - height, h_s - height + col_height);
        pass->filter(*cloud_s);
        serial_mu.lock();
        T = t;
        serial_mu.unlock();
        std::cout << "receive: ";
        outfile << "receive: ";
        for (const auto i : T) {
            std::cout << i << " ";
            outfile << i << " ";
        }
        std::cout << std::endl;
        // 开启桶拟合多线程
        boost::thread_group col_threads;
        col_threads.create_thread([this, &cloud_h] { column_fitting(cloud_h, COLUMN_CLASS::HIGH); });
        col_threads.create_thread([this, &cloud_l] { column_fitting(cloud_l, COLUMN_CLASS::LOW); });
        col_threads.create_thread([this, &cloud_s] { column_fitting(cloud_s, COLUMN_CLASS::STATIC); });
        col_threads.join_all();
#ifndef RELEASE_COLUMN
        viewer.showCloud(cloud_s);
#endif
        // 开启坐标系转换线程
        boost::atomic<int> col_couple_num;
        boost::atomic<bool> col_couple_static;
        col_couple_num = 0;
        col_couple_static = false;
        std::vector<cv::Point2f> col_match(7, {0, 0});
#pragma omp parallel for
        for (uint8_t i = 0; i < 5; i++) {
            if (col_couples[i].size() == 1) {
                switch (col_couples[i].back().color) {
                    case RED:
                        col_r[i] = lidar2tr(col_couples[i].back().point);
                        if (i > 2) {
                            if (flag == SerialApp::RECEIVE_FLAG::BLUE) {
                                col_b[i] = world2tr(cv::Point2f(-560, 0) + lidar2world(col_couples[i].back().point, T),
                                                    T);
                            }
                            if (flag == SerialApp::RECEIVE_FLAG::RED) {
                                col_b[i] = world2tr(cv::Point2f(560, 0) + lidar2world(col_couples[i].back().point, T),
                                                    T);
                            }
                            if (i == 3) {
                                col_match[4] = col_r[i];
                                col_couple_num++;
                            } else {
                                col_match[6] = col_r[i];
                                col_couple_num++;
                            }
                        }
                        break;
                    case BLUE:
                        col_b[i] = lidar2tr(col_couples[i].back().point);
                        if (i > 2) {
                            if (flag == SerialApp::RECEIVE_FLAG::BLUE) {
                                col_r[i] = world2tr(cv::Point2f(560, 0) + lidar2world(col_couples[i].back().point, T),
                                                    T);
                            } else if (flag == SerialApp::RECEIVE_FLAG::RED) {
                                col_r[i] = world2tr(cv::Point2f(-560, 0) + lidar2world(col_couples[i].back().point, T),
                                                    T);
                            }
                            if (i == 3) {
                                col_match[3] = col_b[i];
                                col_couple_num++;
                            } else {
                                col_match[5] = col_b[i];
                                col_couple_num++;
                            }
                        }
                        break;
                    case 255:
                        std::cout << "interfere detected!" << std::endl;
                        break;
                }
            } else if (col_couples[i].size() > 1) {
//                printf("breakpoint2.1\n");
                // 为了防止先识别后面被挡住的桶而找错颜色，先对桶的x进行排序，使之先识别前面的桶
                // sort默认从小到大排序
                sort(col_couples[i].begin(), col_couples[i].end());
                bool match = false;
                std::vector<col_color>::iterator select_itr = col_couples[i].begin(), select_itr_1 =
                        col_couples[i].begin() + 1;
                float best_distance = 0, temp_distance;
                // 注意：end()是最后一个元素的下一个对应的迭代器，不同的数据类型得到的索引结果会不同，有的会段错误有的不会
                for (auto itr = col_couples[i].begin(); itr != col_couples[i].end() - 1; ++itr) {
                    for (auto itr_1 = itr + 1; itr_1 != col_couples[i].end(); ++itr_1) {
                        if ((i < 3 && !near(itr->point, itr_1->point, 660) && near(itr->point, itr_1->point, 740)) ||
                            (i > 2 && !near(itr->point, itr_1->point, 520) && near(itr->point, itr_1->point, 600))) {
                            temp_distance = std::sqrt(std::pow(itr->point.x - itr_1->point.x, 2) +
                                                      std::pow(itr->point.y - itr_1->point.y, 2));
                            if ((i < 3 && std::abs(best_distance - 700) > std::abs(temp_distance - 700)) ||
                                (i > 2 && std::abs(best_distance - 560) > std::abs(temp_distance - 560))) {
                                best_distance = temp_distance;
                                select_itr = itr;
                                select_itr_1 = itr_1;
                                match = true;
                            }
                        }
                    }
                }
                if (match) {
                    if (!is_hidden(select_itr->point, select_itr_1->point)) {
                        if (select_itr->color == select_itr_1->color) {
                            select_itr->color = 255;
                        } else if (select_itr_1->color != 255) {
                            if (select_itr_1->color == RED) {
                                select_itr->color = BLUE;
                            } else {
                                select_itr->color = RED;
                            }
                        }
                    }
                    switch (select_itr->color) {
                        case RED:
                            col_r[i] = lidar2tr(select_itr->point);
                            col_b[i] = lidar2tr(select_itr_1->point);
                            if (i < 3) {
                                col_match[i] = (col_b[i] + col_r[i]) / 2;
                                col_couple_num++;
                            } else if (i == 3) {
                                col_match[3] = col_b[3];
                                col_match[4] = col_r[3];
                                col_couple_static = true;
                                col_couple_num += 2;
                            } else {
                                col_match[5] = col_b[4];
                                col_match[6] = col_r[4];
                                col_couple_static = true;
                                col_couple_num += 2;
                            }
                            break;
                        case BLUE:
                            col_b[i] = lidar2tr(select_itr->point);
                            col_r[i] = lidar2tr(select_itr_1->point);
                            if (i < 3) {
                                col_match[i] = (col_b[i] + col_r[i]) / 2;
                                col_couple_num++;
                            } else if (i == 3) {
                                col_match[3] = col_b[3];
                                col_match[4] = col_r[3];
                                col_couple_static = true;
                                col_couple_num += 2;
                            } else {
                                col_match[5] = col_b[4];
                                col_match[6] = col_r[4];
                                col_couple_static = true;
                                col_couple_num += 2;
                            }
                            break;
                        case 255:
                            std::cout << "interfere detected!" << std::endl;
                            break;
                    }
                }
            }
        }
        outfile << "col:-b";
        for (const auto &i : col_b) {
            outfile << i << " ";
        }
        outfile << "-r";
        for (const auto &i : col_r) {
            outfile << i << " ";
        }
        if (col_couple_num - col_couple_static > 1) {
            // for(short i = 0; i < 7; ++i)
            //     std::cerr << "col_match[" << i << "]=(" << col_match[i].x << ", " << col_match[i].y << ")" << std::endl;
            Eigen::MatrixXf A(col_couple_num * 2 - 2, 2);
            Eigen::MatrixXf B(col_couple_num * 2 - 2, 1);
            char last_col = -1;
            unsigned char matrix_index = 0;
            Eigen::MatrixXf yaw(2, 1);
            float x0 = 0, y0 = 0;
            for (uint8_t col_num = 0; col_num < 7; col_num++) {
                if (col_match[col_num] != cv::Point2f(0, 0)) {
                    if (last_col != -1) {
                        // d for delta
                        float dy = col_match[col_num].y - col_match[last_col].y;
                        float dx = col_match[col_num].x - col_match[last_col].x;
                        float world_dy = world_match[col_num].y - world_match[last_col].y;
                        float world_dx = world_match[col_num].x - world_match[last_col].x;
                        A(matrix_index, 0) = -dy;
                        A(matrix_index, 1) = -dx;
                        A(matrix_index + 1, 0) = dx;
                        A(matrix_index + 1, 1) = -dy;
                        B(matrix_index, 0) = world_dx;
                        B(matrix_index + 1, 0) = world_dy;
                        matrix_index += 2;
                    }
                    last_col = col_num;
                }
            }
            // printf("breakpoint3\n");
            yaw = (A.transpose() * A).inverse() * A.transpose() * B;
            //printf("breakpoint3\n");
            // yaw = A.inverse() * B;
            K = pow(yaw(0, 0), 2) + pow(yaw(1, 0), 2);
            std::cerr << "K=" << K << std::endl;
            std::cerr << "col_couple_num:" << col_couple_num << std::endl;
            if (std::abs(K - 1) < 0.05) {
                float temp_x0, temp_y0;
                float temp_yaw = atan2(yaw(1, 0), yaw(0, 0));
                for (uint8_t col_num = 0; col_num < 7; col_num++) {
                    if (col_match[col_num] != cv::Point2f(0, 0)) {
                        temp_x0 = col_match[col_num].y * cos(temp_yaw) + col_match[col_num].x * sin(temp_yaw) +
                                  world_match[col_num].x - startx;
                        temp_y0 = -col_match[col_num].x * cos(temp_yaw) + col_match[col_num].y * sin(temp_yaw) +
                                  world_match[col_num].y - starty;
                        x0 += temp_x0;
                        y0 += temp_y0;
                        //std::cerr << "(x0, y0)=(" << temp_x0 << ", " << temp_y0 << ")" << std::endl;
                    }
                }
                cal_T[0] = x0 / col_couple_num;
                cal_T[1] = y0 / col_couple_num;
                cal_T[2] = temp_yaw * 1800 / M_PI - startyaw;
                outfile << "localization: " << col_couple_num << " " << cal_T[2] << std::endl;
//                for (int i = 0; i < 3; i++) {
//                    if (col_r[i] == cv::Point2f(0, 0) && col_b[i] != cv::Point2f(0, 0)) {
//                        col_r[i] = 2 * world2tr(world_match[i], cal_T) - col_b[i];
//                    } else if (col_b[i] == cv::Point2f(0, 0) && col_r[i] != cv::Point2f(0, 0)) {
//                        col_b[i] = 2 * world2tr(world_match[i], cal_T) - col_r[i];
//                    }
//                }
                std::cout << "calculate: ";
                for (const auto i : cal_T) {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
            }
        }

        switch (flag) {
            case hitcrt::SerialApp::RED:
//                for (uint8_t i = 0; i < 5; ++i) {
//                    if (col_r[i] != cv::Point2f(0, 0))
//                        if (windows[i].move(tr2world(col_r[i], T)))
//                            col_r[i] = {0, 0};
//                }
                send_col(COLOR_MODE::RED);
                break;
            case hitcrt::SerialApp::BLUE:
//                for (uint8_t i = 0; i < 5; ++i) {
//                    if (col_b[i] != cv::Point2f(0, 0))
//                        if (windows[i].move(tr2world(col_b[i], T)))
//                            col_b[i] = {0, 0};
//                }
                send_col(COLOR_MODE::BLUE);
                break;
            default:
                break;
        }
        gettimeofday(&tv, nullptr);
        std::cout << "--------------"
                  << "cal:" << tv.tv_sec * 1000 + tv.tv_usec / 1000 - ago << std::endl;
        ago = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    } while (!stop_all);
}

void column_identify::stop() {
    stop_all = true;
    thread_grab_.join();
    thread_cal_.join();
    thread_serial_.join();
    outfile.close();
    outfile1.close();
    read_lidar.DeInitLdsLidar();
    std::cout << "----stopped----" << std::endl;
}

int column_identify::detect_color(const cv::Point3f &input) {
    Eigen::Matrix<float, 3, 1> campoint;
    campoint << -input.y - lidar2camx, -input.z - lidar2camy, input.x - lidar2camz;
    campoint = trans_mat * campoint;
    int rgb_x = round(campoint(0, 0) / campoint(2, 0));
    int rgb_y = round(campoint(1, 0) / campoint(2, 0));
    if (abs(rgb_x - 640) < 640 && abs(rgb_y - 512) < 512) {
        grab_mu.lock();
        int color = grab_rgb.at<uchar>(rgb_y, rgb_x);
        // cv::circle(grab_rgb, cv::Point(rgb_x, rgb_y), 5, 150, -1);
        // cv::imshow("detect", grab_rgb);
        grab_mu.unlock();
        // cv::waitKey(1);
        return color;
    }
    //std::cout << "out range!" << std::endl;
    return 255;
}

cv::Point2f column_identify::lidar2tr(const cv::Point2f &input) const {
    return {input.x + lidar_x, input.y + lidar_y};
}

cv::Point2f column_identify::lidar2world(cv::Point2f input, std::vector<float> m) {
    float yaw = (m[2] + startyaw) * M_PI / 1800;
    float x = m[0];
    float y = m[1];
    input = lidar2tr(input);
    return {-input.y * cos(yaw) - input.x * sin(yaw) + x + startx,
            input.x * cos(yaw) - input.y * sin(yaw) + y + starty};
}

cv::Point2f column_identify::tr2world(cv::Point2f input, std::vector<float> m) {
    float yaw = (m[2] + startyaw) * M_PI / 1800;
    float x = m[0];
    float y = m[1];
    return {-input.y * cos(yaw) - input.x * sin(yaw) + x + startx,
            input.x * cos(yaw) - input.y * sin(yaw) + y + starty};
}

cv::Point2f column_identify::world2tr(cv::Point2f input, std::vector<float> m) {
    float yaw = (m[2] + startyaw) * M_PI / 1800;
    float x = m[0];
    float y = m[1];
    input.x = input.x - startx - x;
    input.y = input.y - starty - y;
    return {input.y * cos(yaw) - input.x * sin(yaw),
            -input.x * cos(yaw) - input.y * sin(yaw)};
}

bool column_identify::near(const cv::Point2f &A, const cv::Point2f &B, float distance) {
    float dis = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
    if (dis < distance) {
        return true;
    }
    return false;
}

void column_identify::send_col(COLOR_MODE mode) {
    std::vector<float> data;
    switch (mode) {
        case BLUE:
            if (std::abs(T[2]-cal_T[2])>50)
                cal_T[2] = 3000;
            data = {-col_b[0].y, col_b[0].x, -col_b[1].y, col_b[1].x, -col_b[2].y, col_b[2].x, -col_b[3].y, col_b[3].x,
                    -col_b[4].y, col_b[4].x, cal_T[2]};
            std::cout << "serial: ";
            for (const auto i : data) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            serial.send(hitcrt::SerialApp::SEND_FLAG::COLUMN, data);
            break;
        case RED:
            if (std::abs(T[2]-cal_T[2]>50))
                cal_T[2] = 3000;
            data = {-col_r[0].y, col_r[0].x, -col_r[1].y, col_r[1].x, -col_r[2].y, col_r[2].x, -col_r[3].y, col_r[3].x,
                    -col_r[4].y, col_r[4].x, cal_T[2]};
            std::cout << "serial: ";
            for (const auto i : data) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            serial.send(hitcrt::SerialApp::SEND_FLAG::COLUMN, data);
            break;
    }
}

bool column_identify::is_hidden(const Point2f &A, const Point2f &B) {
    double distance = 2 * std::abs(A.x * B.y - A.y * B.x) / sqrt(pow(A.x + B.x, 2) + pow(A.y + B.y, 2));
    if (distance < 250) {
        return true;
    }
    return false;
}

column_identify::~column_identify() = default;
