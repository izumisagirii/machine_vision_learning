//
// Created by cyh on 2020/12/26.
//
#include<opencv2/opencv.hpp>
#include<iostream>
#include <vector>
using namespace cv;
using namespace std;
char name[100];
vector<Mat> frames;
int main()
{
    VideoCapture capture;
    Mat frame;
    int i = 0;
    capture.open("/home/ew/ba.mkv");
    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    namedWindow("output", WINDOW_NORMAL);
    while (capture.read(frame))
    {
        resize(frame,frame,Size(frame.cols/10,frame.rows/10));
        imshow("output", frame);
        i++;
//        sprintf(name,"../ba/%d.jpg",i);
//        imwrite(name,frame);
        cvtColor(frame,frame,COLOR_BGR2GRAY);
        frames.push_back(frame);
        waitKey(1);
    }
    capture.release();
    capture.open("/home/ew/ba.mkv");
    int j=0;
    while (capture.read(frame))
    {
        j++;
        cvtColor(frame,frame,COLOR_BGR2GRAY);
        for(size_t col=1;col<=10;col++)
        {
            for(size_t row=1;row<=10;row++)
            {
                Mat frame_cal(frame,Rect((col-1)*(frame.cols/10),(row-1)*(frame.rows/10),frame.cols/10,frame.rows/10));
                int flag = 1;
                int diff = 1e8+7;
                for(int t = 1;t<=i;t++)
                {
                    //sprintf(name,"../ba/%d.jpg",t);
                    //Mat frame_diff = imread(name, IMREAD_GRAYSCALE );
                    Mat frame_diff = frames[t-1].clone();
                    absdiff(frame_diff,frame_cal,frame_diff);
                    auto sumdiff = sum(frame_diff);
                    if(sumdiff[0]<diff)
                    {
                        flag = t;
                        diff = sumdiff[0];
                    }
                    if(diff==0)
                    {
                        break;
                    }
                    if(diff>1000000)
                    {
                        t+=5;
                    }
                }
                cout<<j<<": "<<flag<<" "<<diff<<endl;
                //sprintf(name,"../ba/%d.jpg",flag);
                //Mat frame_tran = imread(name, IMREAD_GRAYSCALE );
                Mat frame_tran = frames[flag-1];
                frame_tran.copyTo(frame(Rect((col-1)*(frame.cols/10),(row-1)*(frame.rows/10),frame.cols/10,frame.rows/10)));
                imshow("output", frame);
                waitKey(1);
            }
        }
        sprintf(name,"../out/%d.png",j);
        imwrite(name,frame);
    }
    return 0;
}
