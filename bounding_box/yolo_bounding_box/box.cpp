#include "box.h"

/**
 * @brief 以打开相机或者ONI的方式启动找箭程序
 * @param center 大一统center类的一个对象
 * @param yaml 读入的参数文件，是二值化时三通道的权重
 * @param names YOLO训练时使用的names
 * @param cfg YOLO训练的配置文件
 * @param weights YOLO训练出来的权重文件
 **/
hitcrt::box::box(hitcrt::center& center, const std::string yaml, const std::string names, const std::string cfg, const std::string weights) : Detector(cfg, weights)
{
    if(names.empty() || cfg.empty() || weights.empty())
    {
        std::cerr << "The directory should not be empty!" << std::endl;
        exit(-1);
    }
    this->center = &center;
    std::ifstream ifs_names(names);
    yaml_reader.open(yaml, cv::FileStorage::READ);
    char *buffer = new char[100];
    if(!ifs_names.is_open())
    {
        std::cerr << "Can not open the names file!" << std::endl;
        exit(-1);
    }
    if(!yaml_reader.isOpened())
    {
        std::cerr << "Can not open the yaml!" << std::endl;
        std::cerr << "Load the default params in blue field..." << std::endl;
        feather_param[0] = -1.4;
        feather_param[1] = 1.0;
        feather_param[2] = 1.2;
        head_param[0] = 0.2;
        head_param[1] = 0.2;
        head_param[2] = 1.0;
        shaft_param[0] = 0.1;
        shaft_param[1] = 0.22;
        shaft_param[2] = 0.6;
    }
    else
    {
        if(strcmp(((std::string)yaml_reader["curr_field"]).data(), "red"))
        {
            feather_param[0] = yaml_reader["red_field"]["feather"]["b"];
            feather_param[1] = yaml_reader["red_field"]["feather"]["g"];
            feather_param[2] = yaml_reader["red_field"]["feather"]["r"];
            head_param[0] = yaml_reader["red_field"]["head"]["b"];
            head_param[1] = yaml_reader["red_field"]["head"]["g"];
            head_param[2] = yaml_reader["red_field"]["head"]["r"];
            shaft_param[0] = yaml_reader["red_field"]["shaft"]["b"];
            shaft_param[1] = yaml_reader["red_field"]["shaft"]["g"];
            shaft_param[2] = yaml_reader["red_field"]["shaft"]["r"];
        }
        else
        {
            feather_param[0] = yaml_reader["blue_field"]["feather"]["b"];
            feather_param[1] = yaml_reader["blue_field"]["feather"]["g"];
            feather_param[2] = yaml_reader["blue_field"]["feather"]["r"];
            head_param[0] = yaml_reader["blue_field"]["head"]["b"];
            head_param[1] = yaml_reader["blue_field"]["head"]["g"];
            head_param[2] = yaml_reader["blue_field"]["head"]["r"];
            shaft_param[0] = yaml_reader["blue_field"]["shaft"]["b"];
            shaft_param[1] = yaml_reader["blue_field"]["shaft"]["g"];
            shaft_param[2] = yaml_reader["blue_field"]["shaft"]["r"];
        }
    }
    while(!ifs_names.eof())
    {
        ifs_names.getline(buffer, 100);
        this->names.emplace_back(buffer);
    }
}
/**
 * @brief 以打开视频方式启动找箭程序
 * @param video_file 视频文件路径
 * @param yaml 读入的参数文件，是二值化时三通道的权重
 * @param names YOLO训练时使用的names
 * @param cfg YOLO训练的配置文件
 * @param weights YOLO训练出来的权重文件
 **/
hitcrt::box::box(const std::string video_file, const std::string yaml, const std::string names, const std::string cfg, const std::string weights) : Detector(cfg, weights), video_file(video_file)
{
    if(video_file.empty() || names.empty() || cfg.empty() || weights.empty())
    {
        std::cerr << "The directory should not be empty!" << std::endl;
        return;
    }
    this->center = nullptr;
    // this->video_file = video_file;
    std::ifstream ifs_names(names);
    yaml_reader.open(yaml, cv::FileStorage::READ);
    char *buffer = new char[100];
    if(!ifs_names.is_open())
    {
        std::cerr << "Can not open the names file!" << std::endl;
    }
    if(!yaml_reader.isOpened())
    {
        std::cerr << "Can not open the yaml!" << std::endl;
        std::cerr << "Load the default params in blue field..." << std::endl;
        feather_param[0] = -1.4;
        feather_param[1] = 1.0;
        feather_param[2] = 1.2;
        head_param[0] = 0.2;
        head_param[1] = 0.2;
        head_param[2] = 1.0;
        shaft_param[0] = 0.1;
        shaft_param[1] = 0.22;
        shaft_param[2] = 0.6;
        // These parameters are appropriate in videos recorded by phone, but it is not working here.
    }
    else
    {
        if(strcmp(((std::string)yaml_reader["curr_field"]).data(), "red"))
        {
            feather_param[0] = yaml_reader["red_field"]["feather"]["b"];
            feather_param[1] = yaml_reader["red_field"]["feather"]["g"];
            feather_param[2] = yaml_reader["red_field"]["feather"]["r"];
            head_param[0] = yaml_reader["red_field"]["head"]["b"];
            head_param[1] = yaml_reader["red_field"]["head"]["g"];
            head_param[2] = yaml_reader["red_field"]["head"]["r"];
            shaft_param[0] = yaml_reader["red_field"]["shaft"]["b"];
            shaft_param[1] = yaml_reader["red_field"]["shaft"]["g"];
            shaft_param[2] = yaml_reader["red_field"]["shaft"]["r"];
        }
        else
        {
            feather_param[0] = yaml_reader["blue_field"]["feather"]["b"];
            feather_param[1] = yaml_reader["blue_field"]["feather"]["g"];
            feather_param[2] = yaml_reader["blue_field"]["feather"]["r"];
            head_param[0] = yaml_reader["blue_field"]["head"]["b"];
            head_param[1] = yaml_reader["blue_field"]["head"]["g"];
            head_param[2] = yaml_reader["blue_field"]["head"]["r"];
            shaft_param[0] = yaml_reader["blue_field"]["shaft"]["b"];
            shaft_param[1] = yaml_reader["blue_field"]["shaft"]["g"];
            shaft_param[2] = yaml_reader["blue_field"]["shaft"]["r"];
        }
    }
    while(!ifs_names.eof())
    {
        ifs_names.getline(buffer, 100);
        this->names.emplace_back(buffer);
    }
}
/**
 * @brief 初始化box类探测全过程，用于开启线程
 * @param flag setup的状态，由SETUP_FLAG枚举类型提供
 * @return 无
 **/
void hitcrt::box::setup(int flag = SETUP_FLAG::DEBUG_DETECTOR)
{
    if(center != nullptr)
        center->init();
    // if(flag == SETUP_FLAG::DEBUG_DETECTOR)
    //     debug_runing();
    // else
    //     release_runing();
    if(flag == SETUP_FLAG::DEBUG_DETECTOR)
        _thread = boost::thread([this]{debug_runing();});
    else
        _thread = boost::thread([this]{release_runing();});
    char key = 0;
    system("stty -icanon");
    while(key != 'q')
    {
        usleep(1000);
        key = getchar();
    }
    return;
}
/**
 * @brief debug版的识别箭杆和发送线程，会输出找到箭杆的结果
 * @attention 图像可能会卡顿，属于正常现象，是由于没有找到箭杆时图像不会更新导致的\n
 * 此版本的窗口输出偶尔会出现窗口崩溃，暂时没有解决
 * @return 无
 **/
void hitcrt::box::debug_runing()
{
    if(center != nullptr)
    {
        do
        {
            frame = center->get_rgb_img();
            depth_frame = center->get_depth_img();
            center_points.clear();
            shaft_points.clear();
            robot_center_points.clear();
            robot_shaft_points.clear();
            box_detect();
            if(!bounded_boxes.empty())
            {
                this->arrow_detect();
                this->draw_boxes();
                this->print_info();
                cv::imshow("result", copied_frame);
                cv::waitKey(1);
            }
        }while(!center->stop);
    }else
    {
        cv::VideoCapture video(video_file);
        cv::Mat temp_frame;
        int frame_num = video.get(cv::CAP_PROP_FRAME_COUNT);
        video >> temp_frame;
        frame = std::make_shared<cv::Mat>(temp_frame);
        curr_resolution[0] = frame->cols;
        curr_resolution[1] = frame->rows;
        cv::Mat mask = cv::Mat::zeros(frame->size(), CV_8U);
        // clock_t start, end, sum=0;
        // size_t cnt = 0;
        for(int i = 1; i < frame_num; ++i)
        {
            // cv::imshow("origin", *frame);
            if(frame->empty())
                break;
            box_detect();
            if(!bounded_boxes.empty())
            {
                image_ROI = mask(cv::Rect(bounded_boxes[0].x, bounded_boxes[0].y,
                                    std::min<int>(bounded_boxes[0].w, curr_resolution[0] - 1 - bounded_boxes[0].x),
                                    std::min<int>(bounded_boxes[0].h, curr_resolution[1] - 1 - bounded_boxes[0].y)));
                image_ROI = 1;
            }
            if(!bounded_boxes.empty())
            {
                // start = clock();
                arrow_shaft(mask);
                // {
                //     cnt++;
                //     end = clock();
                //     sum += end-start;
                // }
            }
            cv::imshow("result", *frame);
            cv::waitKey(1);
            video >> temp_frame;
            frame = std::make_shared<cv::Mat>(temp_frame);
            mask = cv::Mat::zeros(frame->size(), CV_8U);
        }
        // std::cout << "sum time: " << (double)sum/(CLOCKS_PER_SEC*cnt) << std::endl;
        // std::cout << "success rate: " << (double)cnt/(frame_num-1) << std::endl;
    }
    cv::destroyAllWindows();
}
/**
 * @brief release版的识别箭杆和发送线程，会输出找到箭杆的结果
 * @attention release版的速度比debug版快至少2倍，主要是省去了输出图像的时间
 * @return 无
 **/
void hitcrt::box::release_runing()
{
    if(center != nullptr)
    {
        do
        {
            frame = center->get_rgb_img();
            depth_frame = center->get_depth_img();
            center_points.clear();
            shaft_points.clear();
            robot_center_points.clear();
            robot_shaft_points.clear();
            box_detect();
            if(!bounded_boxes.empty())
            {
                this->arrow_detect();
                this->print_info();
            }
        }while(!center->stop);
    }else
    {
        cv::VideoCapture video(video_file);
        cv::Mat temp_frame;
        int frame_num = video.get(cv::CAP_PROP_FRAME_COUNT);
        video >> (*frame);
        curr_resolution[0] = frame->cols;
        curr_resolution[1] = frame->rows;
        cv::Mat mask = cv::Mat::zeros(frame->size(), CV_8U);
        for(int i = 1; i < frame_num; ++i)
        {
            // cv::imshow("origin", *frame);
            if(frame->empty())
                break;
            box_detect();
            if(!bounded_boxes.empty())
            {
                image_ROI = mask(cv::Rect(bounded_boxes[0].x, bounded_boxes[0].y,
                                    std::min<int>(bounded_boxes[0].w, curr_resolution[0] - 1 - bounded_boxes[0].x),
                                    std::min<int>(bounded_boxes[0].h, curr_resolution[1] - 1 - bounded_boxes[0].y)));
                image_ROI = 1;
            }
            if(!bounded_boxes.empty())
            {
                arrow_shaft(cv::Mat::ones((*frame).size(), CV_8U));
            }
            cv::imshow("result", *frame);
            cv::waitKey(1);
            video >> (*frame);
            mask = cv::Mat::zeros(frame->size(), CV_8U);
        }
    }
    
    // std::cout << "end of bounding box" << std::endl;
}
/**
 * @brief 对程序的多线程以及大一统类center的对象进行退出，并将文件打开释放
 * @attention 一定要记得在程序中加入这个函数，否则无法正常退出程序可能导致相机连不上USB
 * @attention 之后可能会将其改为析构函数，还未测试
 * @return 无
 **/
void hitcrt::box::shutdown()
{
    center->shutdown();
    yaml_reader.release();
    // join函数防止进程结束后线程依然在运行的bug
    if(_thread.joinable())
        _thread.join();
    return;
}
/**
 * @brief 调用darknet接口获得框选信息bbox_t
 * @return 无
 **/
void hitcrt::box::box_detect()
{
    this->bounded_boxes.clear();
    auto image_ptr = mat_to_image_resize(*frame);
    this->bounded_boxes = detect_resized(*image_ptr, frame->cols, frame->rows);
    return;
}

/**
 * @brief 绘制boxes和中心点坐标
 * @return 无
 **/
void hitcrt::box::draw_boxes()
{
    copied_frame = frame->clone();
    cv::Rect2d result;
    cv::Point center;
    int x, y;
    int cnt = 0;
    cv::Point2f center_point;
    for(auto &i : bounded_boxes)
    {
        if(i.prob < LOWEST_PROB)
        { // 概率太低的框忽略
            continue;
        }
        if(!center_points.empty())
            center_point = center_points[cnt++];
        else
        {
            if(shaft_points[cnt+1].x == 0)
                center_point = shaft_points[cnt++];
            else if(shaft_points[cnt].x == 0)
                center_point = shaft_points[++cnt];
            else
                center_point = (shaft_points[cnt] + shaft_points[++cnt])/2;
            cnt++;
        }
        result.x      = i.x;
        result.y      = i.y;
        result.width  = i.w;
        result.height = i.h;
        center.x      = int(result.x + result.width / 2);
        center.y      = int(result.y + result.height / 2);
        // 这一长串的代码是在保证boundingbox以1.2倍识别区域大小框选
        x = int(center.x - result.width / 2 * 1.2);
        y = int(center.y - result.height / 2 * 1.2);
        if (x <= 0)
            result.x = 0;
        else
            result.x = x;
        if (y <= 0)
            result.y = 0;
        else
            result.y = y;
        if (x + 1.2 * i.w >= frame->cols)
            result.width = frame->cols - x;
        else
            result.width = 1.2 * i.w;
        if (y + 1.2 * i.h >= frame->rows)
            result.height = frame->rows - y;
        else
            result.height = 1.2 * i.h;
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(copied_frame, cv::Rect2d(i.x, i.y, i.w, i.h), color, 3);
        cv::circle(copied_frame, center_point, 5, cv::Scalar(240, 20, 255), 2);
        std::string prob = std::to_string(i.prob);
        cv::putText(copied_frame, this->names[i.obj_id], cv::Point2f(i.x, i.y + i.h), cv::FONT_HERSHEY_SIMPLEX, 1,
                    cv::Scalar(255, 255, 255), 2);
        cv::putText(copied_frame, prob, cv::Point2f(i.x, i.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    }
}

/**
 * @brief 在已框选的区域内寻找箭的箭头和箭杆或者中心位置，使得定位更加精确
 * @return 无
 **/
void hitcrt::box::arrow_detect()
{
    if(!center_points.empty())
        center_points.clear();
    if(!shaft_points.empty())
        shaft_points.clear();
    if(!robot_center_points.empty())
        robot_center_points.clear();
    if(!robot_shaft_points.empty())
        robot_shaft_points.clear();
    float sum_x = 0, sum_y = 0;
    float depth = 0;
    float depth2 = 0;
    cv::Mat mask = cv::Mat::zeros(frame->size(), CV_8U);
    // std::cout << "bounded_boxes size:" << bounded_boxes.size() << std::endl;
    for(size_t i = 0; i < bounded_boxes.size(); i++)
    {
        keypoints.clear();
        if (bounded_boxes[i].prob < LOWEST_PROB)
        { // 把概率太低的box视为无效框
            continue;
        }
        // 这里的ROI按照函数文档注释来看是必须为8bit整形类型的，也就是8U类型
        // 两个min函数防止越界
        rect_ROI = cv::Rect(bounded_boxes[i].x, bounded_boxes[i].y,
                            std::min<int>(bounded_boxes[i].w, curr_resolution[0] - 1 - bounded_boxes[i].x),
                            std::min<int>(bounded_boxes[i].h, curr_resolution[1] - 1 - bounded_boxes[i].y));
        image_ROI = mask(rect_ROI);
        image_ROI = 1;
        if(!arrow_shaft(mask))
        {
            feature_detector->detect(*frame, keypoints, mask);
            if(!keypoints.empty())
            {
                sum_x = 0, sum_y = 0;
                for(cv::KeyPoint point : keypoints)
                {
                    sum_x += point.pt.x;
                    sum_y += point.pt.y;
                }
                // std::cout << "sum_x:" << sum_x << "sum_y" << sum_y << "keypoints.size():" << keypoints.size() << std::endl;
                temp_point = (cv::Mat_<float>(3,1) << sum_x / keypoints.size(), sum_y / keypoints.size(), 1);
                center_points.emplace_back(sum_x / keypoints.size(), sum_y / keypoints.size());
                // Notice: image.at<uchar>(int row, int col)
                depth = (depth_frame->at<ushort>(sum_y / keypoints.size(), sum_x / keypoints.size()))/1000.0;
            }else
            {
                temp_point = (cv::Mat_<float>(3,1) << 
                                std::min<int>(bounded_boxes[i].x + bounded_boxes[i].w / 2, curr_resolution[0] - 1),
                                std::min<int>(bounded_boxes[i].y + bounded_boxes[i].h / 2, curr_resolution[1] - 1), 1);
                center_points.emplace_back(
                    cv::Point2f(std::min<int>(bounded_boxes[i].x + bounded_boxes[i].w / 2, curr_resolution[0] - 1),
                                std::min<int>(bounded_boxes[i].y + bounded_boxes[i].h / 2, curr_resolution[1] - 1)));
                depth = (depth_frame->at<ushort>(std::min<int>(bounded_boxes[i].y + bounded_boxes[i].h / 2, curr_resolution[1] - 1)),
                                                std::min<int>(bounded_boxes[i].x + bounded_boxes[i].w / 2, curr_resolution[0] - 1))/1000.0;
            }
            /** If the speed is not enough, these lines will be replaced by eigen. **/
            center->get_robot_coordinate_point(temp_point, depth);
            robot_center_points.push_back(-temp_point.at<float>(0,0));
            robot_center_points.push_back(temp_point.at<float>(1,0));
        }
        else
        {
            temp_point = (cv::Mat_<float>(3,1) << shaft_points[0].x, shaft_points[0].y, 1);
            temp_point2 = (cv::Mat_<float>(3,1) << shaft_points[1].x, shaft_points[1].y, 1);
            depth = (depth_frame->at<ushort>(shaft_points[0].y, shaft_points[0].x))/1000.0;
            depth2 = (depth_frame->at<ushort>(shaft_points[1].y, shaft_points[1].x))/1000.0;
            center->get_robot_coordinate_point(temp_point, depth);
            center->get_robot_coordinate_point(temp_point2, depth2);
            robot_shaft_points.push_back(-temp_point.at<float>(0,0));
            robot_shaft_points.push_back(temp_point.at<float>(1,0));
            robot_shaft_points.push_back(-temp_point2.at<float>(0,0));
            robot_shaft_points.push_back(temp_point2.at<float>(1,0));
        }
        mask = cv::Mat::zeros(frame->size(), CV_8U);
    }
    // Even though when using center class without serial, we can still send, this will not affect anything.
    if(!robot_center_points.empty())
        center->send(hitcrt::center::SEND_FLAG::ARROW, robot_center_points);
    else if(!robot_shaft_points.empty())
        center->send(hitcrt::center::SEND_FLAG::ARROW, robot_shaft_points);
    /* 下面这部分代码用于测试时显示当前帧的特征点所在位置 */
    // cv::Mat outimg;
    // for (size_t i = 0; i < bounded_boxes.size(); i++)
    // {
    //     cv::drawKeypoints(frame, keypoints_vec[i], outimg);
    // }
    // imshow("output of feature detect", outimg);
    return ;
}

/**
 * @brief 打印框的坐标点，打印框中箭中心或者箭头箭尾的坐标点
 * @return 无
 **/
void hitcrt::box::print_info()
{
    // 当当前帧没有识别到任何东西的时候，name将被赋值为"null"
    unsigned char cnt1 = 0;
    unsigned char cnt2 = 0;
    for(bbox_t temp_box : bounded_boxes)
    {
        std::string name = get_curr_name(temp_box);
        if(name == "null")
        {
            std::cout << std::endl;
            return;
        }
        if(temp_box.prob < LOWEST_PROB)
        {
            continue;
        }
        std::cout << name << ":" << std::endl;
        std::cout << "points:"
                  << "(" << temp_box.x << "," << temp_box.y << ")" << '\t';
        std::cout << "points:"
                  << "(" << temp_box.x + temp_box.w << "," << temp_box.y << ")" << '\t';
        std::cout << "points:"
                  << "(" << temp_box.x + temp_box.w << "," << temp_box.y + temp_box.h << ")" << '\t';
        std::cout << "points:"
                  << "(" << temp_box.x << "," << temp_box.y + temp_box.h << ")" << '\n';
    }
    for(unsigned char i = 0; i < center_points.size(); ++i)
    {
        std::cout << "center point:"
                  << "(" << center_points[i].x << "," << center_points[i].y << ")" << std::endl;
        std::cout << "robot coordinate center point:"
                  << "(" << -robot_center_points[cnt1++];
        std::cout << "," << robot_center_points[cnt1++] << ")" << std::endl;
    }
    for(unsigned char i = 0; i < shaft_points.size(); i+=2)
    {
        std::cout << "arrowhead:"
                  << "(" << shaft_points[i].x << "," << shaft_points[i].y << ")" << std::endl;
        std::cout << "feather:"
                  << "(" << shaft_points[i+1].x << "," << shaft_points[i+1].y << ")" << std::endl;
        std::cout << "robot coordinate arrowhead:"
                  << "(" << -robot_shaft_points[cnt2++];
        std::cout << "," << robot_shaft_points[cnt2++] << ")" << std::endl;
        std::cout << "robot coordinate feather:"
                  << "(" << -robot_shaft_points[cnt2++];
        std::cout << "," << robot_shaft_points[cnt2++] << ")" << std::endl;
    }
}

/**
 * @brief 找到箭杆以及确定哪一端是箭头哪一端是箭尾
 * @attention 这个方法中的二值化图是通过三通道分离后求加权和的结果进行的，所以对于不同场地要有不同参数
 * @attention 这个方法找到的箭杆在近距离下的方向倾斜角度与箭杆杆柱中心的直线角度有较大差距
 * @param mask 图像掩膜，具体到找箭中就是darknet得到的目标框生成的掩膜 
 * @return 判断是否成功找出箭杆
 **/
bool hitcrt::box::arrow_shaft(cv::Mat mask)
{
    cv::Mat img/*, kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5))*/;
    cv::Mat arrowBW_INV/*, arrowBW_INV2*/;
    cv::Mat arrow;
    // clock_t start = clock();
    cv::cvtColor(*frame, arrow, cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> channels;
    cv::split(*frame, channels);
	cv::threshold(shaft_param[0]*channels[0]+shaft_param[1]*channels[1]+shaft_param[2]*channels[2], arrowBW_INV, 130, 255, cv::THRESH_BINARY_INV);
    // cv::imshow("channel3",feather_param[0]*channels[0]+feather_param[1]*channels[1]+feather_param[2]*channels[2]);
    // std::cout << int(cv::Mat(feather_param[0]*channels[0]+feather_param[1]*channels[1]+feather_param[2]*channels[2]).at<uchar>(320, 120)) << std::endl;
    // cv::threshold(feather_param[0]*channels[0]+feather_param[1]*channels[1]+feather_param[2]*channels[2], arrowBW_INV2, 130, 255, cv::THRESH_BINARY);
    // cv::threshold(arrow, arrowBW_INV, 90, 255, cv::THRESH_BINARY_INV);
    // cv::threshold(arrow, arrowBW_INV2, 110, 255, cv::THRESH_BINARY_INV);
    // cv::dilate(arrowBW_INV, arrowBW_INV, kernel);
    // clock_t end = clock();
    // double thre_time = (double)(end - start) / CLOCKS_PER_SEC;
    // std::cout << "threshold time=" << thre_time << std::endl;
    // start = clock();
    cv::Mat out, stats, centroids;
    arrowBW_INV.copyTo(img, mask);
    // cv::imshow("img", img);
    // arrowBW_INV2.copyTo(arrowBW_INV2, mask);
	int number(cv::connectedComponentsWithStats(img, out, stats, centroids, 8, CV_16U));
    // end = clock();
    // double contours_time = (double)(end - start) / CLOCKS_PER_SEC;
    // std::cout << "contours time=" << contours_time << std::endl;
    // start = clock();
    unsigned char count = 0;
    if(number <= 1)
        return false;
    cv::Point2f arrowhead, feather;
    /**There is a problem: we need to identify which edge of the shaft is close to feather when we can't see it.**/
    cv::Mat rect_mask1 = img(cv::Rect(rect_ROI.x, rect_ROI.y, rect_ROI.width/2, rect_ROI.height));
    cv::Mat rect_mask2 = img(cv::Rect(rect_ROI.x+rect_ROI.width/2+1, rect_ROI.y,
                              rect_ROI.width%2?rect_ROI.width/2+1:rect_ROI.width/2, rect_ROI.height));
    unsigned short res_and1, res_and2, res_rect1, res_rect2;
    res_rect1 = PixelCounter(rect_mask1);
    res_rect2 = PixelCounter(rect_mask2);
    for (unsigned char i = 1; i < number; ++i) 
    {
        // In fact, when one edge of the arrow is out of the vision field, it will be represented by x=0, y=0.
        int x(stats.at<int>(i, cv::CC_STAT_LEFT));
        int y(stats.at<int>(i, cv::CC_STAT_TOP));
        int w(stats.at<int>(i, cv::CC_STAT_WIDTH));
        int h(stats.at<int>(i, cv::CC_STAT_HEIGHT));
        int area(stats.at<int>(i, cv::CC_STAT_AREA));
        // std::cout << x << " " << y << std::endl;
        if (area > 100) {
            count++;
            cv::Mat kernel1(cv::Mat::zeros(h,w,CV_8UC1));
            cv::Mat kernel2;
            kernel1.copyTo(kernel2);
            /* By testing, we know that the LINE_4 has the same success rate compared with LINE_8, but reduces the time cost by 0.1ms.*/
            cv::line(kernel1, cv::Point(0, 0), cv::Point(w,h), cv::Scalar(255), 6, cv::LINE_4, 0);
            cv::line(kernel2, cv::Point(0,h), cv::Point(w,0), cv::Scalar(255), 6, cv::LINE_4, 0);
            cv::Rect rect(x, y, w, h);
            cv::Mat ROI(img(rect));
            cv::Mat and1, and2;
            cv::bitwise_and(kernel1, ROI, and1);
            cv::bitwise_and(kernel2, ROI, and2);
            res_and1 = PixelCounter(and1);
            res_and2 = PixelCounter(and2);
            if(res_and1 > res_and2&&res_and1 > 80)
            {
                if(count == 1)
                {
                    if(res_rect1 > res_rect2)
                    {
                        arrowhead = cv::Point2f(x+w, y+h);
                        feather = cv::Point2f(x, y);
                    }
                    else
                    {
                        arrowhead = cv::Point2f(x, y);
                        feather = cv::Point2f(x+w, y+h);
                    }
                }
                else
                {
                    if(res_rect1 > res_rect2)
                    {
                        if(x+w > arrowhead.x)
                            arrowhead = cv::Point2f(x+w, y+h);
                        else if(x < feather.x)
                            feather = cv::Point2f(x, y);
                    }
                    else
                    {
                        if(x+w > feather.x)
                            feather = cv::Point2f(x+w, y+h);
                        else if(x < arrowhead.x)
                            arrowhead = cv::Point2f(x, y);
                    }
                }
                cv::line(*frame, arrowhead, feather, cv::Scalar(255, 255, 0), 2, cv::LINE_4, 0);
            }
            else if(res_and1 < res_and2&&res_and2 > 80) 
            {
                if(count == 1)
                {
                    if(res_rect1 > res_rect2)
                    {
                        feather = cv::Point2f(x, y+h);
                        arrowhead = cv::Point2f(x+w, y);
                    }
                    else
                    {
                        feather = cv::Point2f(x+w, y);
                        arrowhead = cv::Point2f(x, y+h);
                    }
                }
                else
                {
                    if(res_rect1 > res_rect2)
                    {
                        if(x+w > arrowhead.x)
                            arrowhead = cv::Point2f(x+w, y);
                        else if(x < feather.x)
                            feather = cv::Point2f(x, y+h);
                    }
                    else
                    {
                        if(x+w > feather.x)
                            feather = cv::Point2f(x+w, y);
                        else if(x < arrowhead.x)
                            arrowhead = cv::Point2f(x, y+h);
                    }
                }
                cv::line(*frame, arrowhead, feather, cv::Scalar(255, 255, 0), 2, cv::LINE_4, 0);
            }
            // This means the line fitting failed.
            else
                return false;
        }
        // std::cout << "number:" << i << ",area:" << area << std::endl;
    }
    // std::cout << "\033[031mcount:\033[01m" << (int)count << std::endl;
    // end = clock();
    // double match_time = (double)(end - start) / CLOCKS_PER_SEC;
    // std::cout << "match time=" << match_time << std::endl;
    if(count > 0)
    {
        shaft_points.emplace_back(arrowhead);
        shaft_points.emplace_back(feather);
        return true;
    }
    else
        return false;
}
/**
 * @brief 数出白色像素点个数
 * @param src 要数白色像素个数的图像
 * @return 白色像素个数
 * @attention 这个函数需要反复执行多次，为避免耗时选择了内联函数的形式
 **/
inline unsigned short hitcrt::box::PixelCounter(cv::Mat& src) {
	unsigned short num = 0;
	auto itor(src.begin<uchar>());
	auto itor_end(src.end<uchar>());
	for (; itor != itor_end; ++itor) {
		if ((*itor) > 0) {
			++num;
		}
	}
	return num;
}