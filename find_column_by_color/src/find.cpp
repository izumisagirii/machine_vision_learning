#include <iostream>
#include<opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define PI 3.1415926
using namespace cv;
using namespace std;
void RGB2HSV(double red, double green, double blue, double& hue, double& saturation, double& intensity)
{
	double r, g, b;
	double h, s, i;
	double sum;
	double minRGB, maxRGB;
	double theta;
	r = red / 255.0;
	g = green / 255.0;
	b = blue / 255.0;
	minRGB = ((r<g) ? (r) : (g));
	minRGB = (minRGB<b) ? (minRGB) : (b);
	maxRGB = ((r>g) ? (r) : (g));
	maxRGB = (maxRGB>b) ? (maxRGB) : (b);
	sum = r + g + b;
	i = sum / 3.0;
	if (i<0.001 || maxRGB - minRGB<0.001)
	{
		h = 0.0;
		s = 0.0;
	}
	else
	{
		s = 1.0 - 3.0*minRGB / sum;
		theta = sqrt((r - g)*(r - g) + (r - b)*(g - b));
		theta = acos((r - g + r - b)*0.5 / theta);
		if (b <= g)
			h = theta;
		else
			h = 2 * PI - theta;
		if (s <= 0.01)
			h = 0;
	}
 
	hue = (int)(h * 180 / PI);
	saturation = (int)(s * 100);
	intensity = (int)(i * 100);
}
 
Mat picture_red(Mat input)
{
	
	Mat frame;
	Mat srcImg = input;
	frame = srcImg;
	waitKey(1);
	int width = srcImg.cols;
	int height = srcImg.rows;
 
	int x, y;
	double B = 0.0, G = 0.0, R = 0.0, H = 0.0, S = 0.0, V = 0.0;
	Mat vec_rgb = Mat::zeros(srcImg.size(), CV_8UC1);
	for (x = 0; x < height; x++)
	{
		for (y = 0; y < width; y++)
		{
			B = srcImg.at<Vec3b>(x, y)[0];
			G = srcImg.at<Vec3b>(x, y)[1];
			R = srcImg.at<Vec3b>(x, y)[2];
			RGB2HSV(R, G, B, H, S, V);
			//红色范围，范围参考的网上。可以自己调
			if ((H >= 312 && H <= 360) && (S >= 17 && S <= 100) && (V>18 && V < 100))
				vec_rgb.at<uchar>(x, y) = 255;
			/*cout << H << "," << S << "," << V << endl;*/
		}
	}
	/*imshow("hsv", vec_rgb);*/
	return vec_rgb;
}
 
Mat mask(Mat image,Mat mask)
{
    // Mat mask(middle.size(),CV_8UC1,cv::Scalar(1));
	// Mat image_masked = image(Rect(0,0,image.cols,image.rows));
	Mat image_masked;
	image.copyTo(image_masked,mask);
	return image_masked;

	// Mat mask(depth.size(),CV_8UC1,cv::Scalar(1));
    // rectangle(mask,rect,cv::Scalar(255),-1);
    // imshow("mask",mask);
    // Mat fgimg;
	// depth.copyTo(fgimg,mask);
}




int main()
{
	double xx1, yy1, xx2, yy2;
	double x1, y1, x2, y2;
 
	Mat matSrc = imread("../red.png");
	
	Mat middle = picture_red(matSrc);
	
	Mat image_masked = mask(matSrc, middle);
	imshow("原图", matSrc);
	imshow("red",middle);
	imshow("masked",image_masked);
	waitKey();
 
	return 0;
}