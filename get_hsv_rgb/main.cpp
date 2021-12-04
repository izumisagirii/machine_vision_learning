#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
#define WINDOW_NAME "pic_hsv"
#define WINDOW_NAME_IN "pic_rgb"
void on_MouseHandle(int event, int x, int y, int flags, void *param);
int main(int argc, char** argv)
{
	Mat inputImage, outputImage, grabImage;
	if(argc == 1)
	{
		//int multiple = 1;					 //图片的放大倍数
		inputImage = imread("../zipped/1.png"); //这里放置自己的文件路径。
		if (inputImage.empty())
		{
			cout << "\033[31m路径错误\033[0m" << endl;
			return 0;
		}
	}
	else
	{
		inputImage = imread(argv[1]);
		if (inputImage.empty())
		{
			cout << "\033[31m路径错误\033[0m" << endl;
			return 0;
		}
	}
	//resize(inputImage, inputImage, Size(multiple * inputImage.cols, multiple * inputImage.rows));
	//resize(grabImage, grabImage, Size(multiple * grabImage.cols, multiple * grabImage.rows));
	//medianBlur ( inputImage, inputImage, 3);
	cvtColor(inputImage, outputImage, COLOR_BGR2HSV);
	// cv::Mat mask_r;
	// cv::Mat mask_b;
	// cv::Mat depth_mask_r;
	// cv::Mat depth_mask_b;
	// // cv::inRange(outputImage,cv::Scalar(95,180,70),cv::Scalar(135,255,255),mask_b);
	// // inputImage.copyTo(depth_mask_b, mask_b);
	// cv::inRange(outputImage,cv::Scalar(0,90,85),cv::Scalar(12,255,255),mask_r);
	// inputImage.copyTo(depth_mask_r, mask_r);
	// cv::inRange(outputImage,cv::Scalar(160,90,85),cv::Scalar(180,255,255),mask_r);
	// inputImage.copyTo(depth_mask_r, mask_r);
	cout << "点击操作ESC退出，rgb顺序 \033[34mB\033[0m\033[32mG\033[0m\033[31mR\033[0m" << endl;
	//设置鼠标操作回调函数
	namedWindow(WINDOW_NAME_IN,cv::WINDOW_NORMAL);
	namedWindow(WINDOW_NAME,cv::WINDOW_NORMAL);
	while (1)
	{
		imshow(WINDOW_NAME_IN, inputImage);
		imshow(WINDOW_NAME, outputImage);
		setMouseCallback(WINDOW_NAME_IN, on_MouseHandle, (void *)&inputImage);
		setMouseCallback(WINDOW_NAME, on_MouseHandle, (void *)&outputImage);
		if (waitKey(10) == 27)
			break; //按下ESC键，程序退出
	}
	waitKey();
	return 0;
}

void on_MouseHandle(int event, int x, int y, int flags, void *param)
{

	Mat &image = *(cv::Mat *)param;
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
	{
		cout << static_cast<int>(image.at<Vec3b>(y, x)[0]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[1]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[2]) << endl;
	}
	break;
	}
}
