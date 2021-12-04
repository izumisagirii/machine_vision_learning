#include "odometry.h"
odometry::odometry(Eigen::Matrix3d depth_K, Eigen::Matrix3d init_R, Eigen::Vector3d init_t)
{
    this->K = (cv::Mat_<double>(3,3) << depth_K(0,0), depth_K(0,1), depth_K(0,2),
                                        depth_K(1,0), depth_K(1,1), depth_K(1,2),
                                        depth_K(2,0), depth_K(2,1), depth_K(2,2));
    this->R = (cv::Mat_<double>(3,3) << init_R(0,0), init_R(0,1), init_R(0,2),
                                             init_R(1,0), init_R(1,1), init_R(1,2),
                                             init_R(2,0), init_R(2,1), init_R(2,2));
    this->t = (cv::Mat_<double>(3,1) << init_t(0), init_t(1), init_t(2));

}
void odometry::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2)
{
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    matcher->match(descriptors_1, descriptors_2, temp_matches);

    double min_dist = 6000, dist;
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        dist = temp_matches[i].distance;
        if(dist < min_dist) min_dist = dist;
    }

    for(int i = 0; i < descriptors_1.rows; i++)
    {
        if(temp_matches[i].distance <= std::max(2*min_dist, 10.0)){
            matches.push_back(temp_matches[i]);
        }
    }
}
/**
 * @brief 前端的全部操作，为了提高运行效率而采用了紧耦合的代码风格
 * 
 * @param aligned_color_img 与深度图匹配好的颜色图
 * 
 * @return 用于判断是否执行成功
 **/
bool odometry::solve(cv::Mat aligned_color_img, cv::Mat depth_img)
{
    pts_1.clear();
    pts_2.clear();
    keypoints_1.clear();
    keypoints_2.clear();
    temp_matches.clear();
    matches.clear();
    // 条件判断：当没有上一帧图像时，先存储当前帧的图像
    if(aligned_color_img_0.empty())
    {
        aligned_color_img_0 = aligned_color_img;
        depth_img_0 = depth_img;
        return false;
    }
    // 条件判断：未更新最新一帧的图像时不进行求解
    if(aligned_color_img.data == aligned_color_img_0.data)
    {
        return false;
    }
    find_feature_matches(aligned_color_img_0, aligned_color_img);

    // 完成特征匹配，开始投出三维点
    for(cv::DMatch m : matches)
    {
        ushort d1 = depth_img_0.ptr<unsigned short>
                (int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if(d1 == 0)
            continue;
        ushort d2 = depth_img.ptr<unsigned short>
                (int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
        if(d2 == 0)
            continue;
        temp_p1 = cv::Point2d((keypoints_1[m.queryIdx].pt.x-K.at<float>(0,2))/K.at<float>(0,0),
                                    (keypoints_1[m.queryIdx].pt.y-K.at<float>(1,2))/K.at<float>(1,1));
        temp_p2 = cv::Point2d((keypoints_2[m.trainIdx].pt.x-K.at<float>(0,2))/K.at<float>(0,0),
                                    (keypoints_2[m.trainIdx].pt.y-K.at<float>(1,2))/K.at<float>(1,1));
        // std::cout << keypoints_1[m.queryIdx].pt.x-K.at<float>(0,2) << K.at<float>(0,0) << std::endl;
        // std::cout << "keypoints_1[m.queryIdx].pt=" << keypoints_1[m.queryIdx].pt << '\t'
        //           << "keypoints_2[m.queryIdx].pt=" << keypoints_2[m.trainIdx].pt << std::endl;
        // std::cout << "K = " << K << std::endl;
        // std::cout << d1 << '\t' << d2 << std::endl;
        float dd1 = float(d1)/1000.0;
        float dd2 = float(d2)/1000.0;
        pts_1.push_back(cv::Point3f(temp_p1.x*dd1, temp_p1.y*dd1, dd1));
        pts_2.push_back(cv::Point3f(temp_p2.x*dd2, temp_p2.y*dd2, dd2));
        // std::cout << pts_1.back() << '\t' << pts_2.back() << std::endl;
        // 求质心用
        center_pts1 += pts_1.back();
        center_pts2 += pts_2.back();
    }
    // 前一帧和后一帧的质心
    int N = pts_1.size();
    center_pts1 = center_pts1/N;
    center_pts2 = center_pts2/N;
    // std::cout << center_pts1 << std::endl
    //           << center_pts2 << std::endl;
    // 对点去质心
    for(int i = 0; i < N; i++)
    {
        pts_1[i] -= center_pts1;
        pts_2[i] -= center_pts2;
    }

    // 计算SVD中的W矩阵
    W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(pts_1[i].x, pts_1[i].y, pts_1[i].z)*
            Eigen::Vector3d(pts_2[i].x, pts_2[i].y, pts_2[i].z).transpose();
    }
    // std::cout << "W = " << W << std::endl;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, 
            Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    // std::cout << "U = " << U << "\nV = " << V << std::endl;
    R_ = U*(V.transpose());
    if(R_.determinant() < 0){
        R_ = -R_;
    }
    t_ = Eigen::Vector3d(center_pts1.x, center_pts1.y, center_pts1.z)
        -R_*Eigen::Vector3d(center_pts2.x, center_pts2.y, center_pts2.z);
    R = (cv::Mat_<double>(3,3) << R_(0,0), R_(0,1), R_(0,2),
                                  R_(1,0), R_(1,1), R_(1,2),
                                  R_(2,0), R_(2,1), R_(2,2));
    t = (cv::Mat_<double>(3,1) << t_(0,0), t_(1,0), t_(2,0));
    aligned_color_img_0 = aligned_color_img;
    depth_img_0 = depth_img;
    return true;
}
void odometry::print_info()
{
    std::cout << "R:\n" << R << std::endl;
    std::cout << "t:\n" << t << std::endl;

}